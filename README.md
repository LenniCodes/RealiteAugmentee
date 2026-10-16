# ReVA — Réalité augmentée avec OpenCV

Projet du cours de réalité augmentée / virtuelle / mixte (Polytech Marseille, 5A).
Sujet : <https://mavromatis.org/ravm/index.html>

Le projet se fait en trois parties :
- **AR** : suivi d'une feuille A4 et incrustation d'objets 3D ;
- **VR** : labyrinthe 3D et environnement virtuel ;
- **MR** : jeu de bille piloté par l'inclinaison de la feuille.

**Objectif de robustesse :** le suivi ne doit pas dépendre d'une feuille A4 blanche bien filmée. Il doit fonctionner sur **n'importe quelle surface rectangulaire**, même mal filmée (voir [Tests de robustesse](#tests-de-robustesse)).

**État actuel :** la partie AR fonctionne en temps réel sur la vidéo du prof et sur les vidéos de synthèse. Un menu permet de choisir la vidéo, et un mode test automatique mesure les performances. Les surfaces autres qu'une feuille A4 ne sont pas encore gérées.

---

## Démarrage rapide

```bash
# Dépendances (Ubuntu / Debian)
sudo apt install build-essential cmake ninja-build libopencv-dev
# ou avec Nix
nix --extra-experimental-features 'nix-command flakes' develop

# Vidéos de test : mettre les vidéos et images dans media/
wget -P media https://mavromatis.org/ravm/input.mp4 https://mavromatis.org/ravm/input.jpg

# Compiler
cmake -S . -B build -G Ninja
cmake --build build

# Lancer (depuis build/)
cd build
./CoursOpenCv                 # menu : choisir une vidéo dans ../media
```

## Utilisation

| Commande | Effet |
|---|---|
| `./CoursOpenCv` | menu avec toutes les vidéos et images de `../media`, plus la webcam |
| `./CoursOpenCv mon_dossier` | menu sur un autre dossier |
| `./CoursOpenCv ../media/input.mp4` | lance directement une vidéo (ou une image, ou `0` pour la webcam) |
| `./CoursOpenCv --bench` | **mode test** : passe sur toutes les vidéos du dossier, sans affichage, et affiche un tableau |
| `./CoursOpenCv --dump ../diag` | comme `--bench`, et enregistre ~10 images de diagnostic par vidéo dans `../diag` |
| `--cam autre.yaml` | autres paramètres de caméra (défaut : `../input_camera.yaml`) |

**Dans le menu :** Haut/Bas ou clic pour choisir, Entrée ou double-clic pour lancer, `1`-`9` pour lancer directement, `u` pour rescanner le dossier, `q` pour quitter. Pour chaque source, le menu affiche un aperçu, la résolution, les fps et la durée.

**Pendant le suivi :**
- `q` / `Échap` : retour au menu
- `n` / `p` : vidéo suivante / précédente
- `Espace` : pause
- `d` : debug (masque seuillé et zone de recherche)
- `r` : relancer la détection

À l'écran s'affichent l'état du suivi, le temps de calcul par image, les fps d'affichage et l'erreur de reprojection.

### Tester le programme

1. Mettre toutes les vidéos à tester dans `media/` : la vidéo et l'image du prof, nos propres vidéos (éclairages variés, angles, occultations), et des vidéos de synthèse.
2. Lancer `./CoursOpenCv --bench`. Exemple de sortie :

```
image_fixe.jpg                1280x720       30 img  suivi 100.0%  perdu   0.0%  algo   2.22 ms (max    4.1)  reproj 0.11 px
synthese_1080p.mp4            1920x1080     300 img  suivi  96.0%  perdu   0.0%  algo   4.18 ms (max   10.1)  reproj 0.51 px
synthese_480p.mp4              854x480      300 img  suivi  98.7%  perdu   0.0%  algo   1.58 ms (max    8.7)  reproj 0.24 px
synthese_720p.mp4             1280x720      400 img  suivi  95.2%  perdu   2.5%  algo   2.47 ms (max    8.6)  reproj 0.29 px
```

- `suivi` : pourcentage d'images où la feuille est détectée et validée.
- `perdu` : pourcentage d'images sans pose disponible.
- `algo` : temps de calcul moyen et maximal par image.
- `reproj` : écart moyen, en pixels, entre les coins détectés et les coins reprojetés par la pose. Plus c'est petit, plus la pose est juste.

3. Pour vérifier à l'œil, lancer le menu et regarder si le cube reste « collé » à la feuille.
4. Pour la webcam : choisir « Webcam 0 » dans le menu.

**Vidéos de synthèse.** L'outil `gen_synthetic` génère une feuille A4 dont on connaît la pose exacte. Elle bouge, s'incline et fait un tour complet sur elle-même, sur un fond bruité, avec un éclairage variable, un objet blanc parasite et deux occultations. Pratique pour tester plusieurs résolutions :

```bash
./gen_synthetic ../input_camera.yaml 1280 720 400 ../media/synthese_720p.mp4
./gen_synthetic ../input_camera.yaml 1920 1080 300 ../media/synthese_1080p.mp4
./gen_synthetic ../input_camera.yaml 854 480 300 ../media/synthese_480p.mp4
```

---

## Fonctionnement (pipeline AR)

```
image ─► zone de recherche ─► sous-résolution ─► Otsu + contours ─► quadrilatère
      ─► coins pleine résolution + sub-pixel ─► ordre cohérent ─► solvePnP (validation)
      ─► lissage One-Euro ─► pose (rvec, tvec) ─► cube + repère
```

1. **Zone de recherche.** Quand la feuille est suivie, on ne cherche qu'autour de sa position précédente, avec une marge de 35 %. Si on la perd, on cherche dans toute l'image.
2. **Sous-résolution.** La détection grossière se fait sur une image réduite à 480 px de large maximum. C'est l'essentiel du gain de vitesse.
3. **Seuillage.** Flou gaussien, puis **seuil d'Otsu**, qui s'adapte automatiquement à l'éclairage, puis ouverture morphologique pour supprimer le bruit.
4. **Choix du quadrilatère.** Parmi les contours, on garde ceux qui forment, après `approxPolyDP`, un quadrilatère convexe aux angles raisonnables et bien rempli (un objet rond est donc rejeté). Quand on suit la feuille, on préfère le candidat le plus proche de la position précédente.
5. **Précision.** Les coins sont remis en pleine résolution, puis affinés au sous-pixel avec `cornerSubPix`.
6. **Ordre des coins.** La première fois, on les trie dans le sens horaire en commençant par un grand côté. Ensuite, chaque coin garde son identité d'une image à l'autre, ce qui évite les sauts de 90° ou 180° du contenu 3D.
7. **Validation.** `solvePnP` (méthode `SOLVEPNP_IPPE`, prévue pour les cibles planes) calcule la pose. Si l'erreur de reprojection dépasse 4 px, le quadrilatère n'a pas la forme d'une A4 vue en perspective et il est rejeté.
8. **Lissage.** Un filtre **One-Euro** est appliqué aux coins. Il lisse fort quand la feuille est immobile (supprime le tremblement) et peu quand elle bouge (pas de retard).
9. **Perte temporaire.** Pendant 5 images, on garde la dernière pose (état `COASTING`). Au-delà, l'état passe à `LOST` et on recherche dans toute l'image.

**Caméra.** `K` et les coefficients de distorsion sont lus dans `input_camera.yaml`. Si la vidéo n'a pas la résolution prévue par le fichier (1280×720), `K` est mis à l'échelle automatiquement. Pour une autre caméra, il suffit de fournir un autre YAML avec `--cam`.

> `solvePnP` ne calcule pas la distorsion : il prend `K` et les coefficients de distorsion **en entrée**.
> Pour calibrer notre propre caméra, il faudra utiliser `cv::calibrateCamera` avec un damier.

Tous les réglages sont dans `TrackerParams` (`include/tracker.hpp`). Les plus importants :
- `procWidth` : vitesse contre fiabilité ;
- `filterMinCutoff` et `filterBeta` : rendu lisse contre réactivité.

## Structure

```
include/
  pose.hpp        CameraParams, chargement du YAML, ordre des coins, solvePnP, dessin du cube
  tracker.hpp     SheetTracker (détection + suivi + lissage), filtre One-Euro, TrackerParams
  menu.hpp        menu de choix des sources
src/
  main.cpp        arguments, menu, session de suivi, mode test (--bench)
  pose.cpp
  tracker.cpp
  menu.cpp
tools/
  gen_synthetic.cpp   générateur de vidéos de test
media/            vidéos et images de test (non versionnées)
input_camera.yaml paramètres intrinsèques fournis par le prof
```

## Résultats

Mesures faites sur une séquence de synthèse 720p de 400 images, où la pose vraie est connue :

| | Ancien (MSER) | Actuel |
|---|---|---|
| Temps de calcul par image | 60 ms | **2,5 ms** |
| Images où la feuille est trouvée | 142 / 382 | **380 / 382** |
| Erreur sur les coins | 1,25 px | **0,95 px** |
| Tremblement d'une image à l'autre | — | 0,43 px (lissé) |
| Fausses détections pendant les occultations | — | 0 |
| Sauts d'orientation (90° / 180°) | — | 0 |

## Tests de robustesse

### Pourquoi une vidéo avec de la mousse

Une feuille A4 blanche, posée sur un fond sombre et filmée bien en face, est le cas le plus facile. Pour savoir si le logiciel est vraiment utilisable, on a aussi filmé volontairement dans de **mauvaises conditions** (`media/VID_20261016_151224.mp4`) :
- **la surface suivie est un rectangle de mousse**, pas une feuille : elle n'a ni la couleur, ni la texture, ni les proportions d'une A4 ;
- **la vidéo est filmée au téléphone, en portrait** (1080×1440), à la main, donc avec du bougé, sous un éclairage quelconque, et avec une caméra différente de celle de `input_camera.yaml`.

Le but est de **trouver les limites de l'algorithme** et de le faire évoluer pour qu'il marche sur **n'importe quelle surface rectangulaire, même mal filmée**. Le futur jeu en dépend : le joueur doit pouvoir utiliser l'objet plat qu'il a sous la main (feuille, livre, plateau, morceau de mousse), sans conditions de tournage idéales.

### Résultats actuels sur données réelles

`./CoursOpenCv --bench`, OpenCV 4.5.4 :

| Source | Résolution | Suivi | Perdu | Calcul | Reprojection |
|---|---|---|---|---|---|
| `input.jpg` (prof) | 640×480 | 100 % | 0 % | 1,4 ms | 0,67 px |
| `input.mp4` (prof) | 1280×720 | 24 % | 71 % | 2,2 ms | 2,7 px |
| `VID_20261016_151224.mp4` (mousse) | 1080×1440 | **0 %** | 100 % | 2,8 ms | — |

La vidéo avec la mousse n'est **pas encore suivie du tout**. C'est un résultat attendu, et c'est justement ce qu'elle doit révéler. L'algorithme actuel repose sur trois hypothèses qu'elle ne respecte pas :

1. **La surface est plus claire que le fond.** Otsu garde la partie claire de l'image. Une surface sombre ou colorée sur un fond clair n'est donc jamais candidate.
2. **La surface est une A4 (297 × 210 mm).** La validation compare le quadrilatère détecté à une A4 vue en perspective, avec l'erreur de reprojection de `solvePnP`. Un rectangle d'autres proportions est rejeté, même s'il est parfaitement détecté.
3. **La caméra est celle de `input_camera.yaml`.** Pour une vidéo de téléphone en portrait, la matrice `K` n'est qu'une approximation, ce qui fausse aussi la validation.

### Pistes pour généraliser

- **Dimensions de la surface en paramètre** (`--size 400x300` en mm), au lieu de l'A4 codée en dur. En option, estimer automatiquement les proportions du rectangle à partir de l'homographie et de `K`.
- **Détection indépendante de la couleur :** tester les deux polarités (surface claire *ou* sombre), ou passer par les contours (Canny et ajustement de droites sur les 4 bords), qui ne dépendent pas de la luminosité de la surface.
- **Validation par la géométrie et la cohérence dans le temps** (quadrilatère convexe, mouvement continu) plutôt que par la seule ressemblance à une A4.
- **Calibrer la caméra du téléphone** avec `calibrateCamera` et un damier, pour ne plus dépendre d'une matrice `K` approximée.
- **Diagnostic :** `--dump` montre pour chaque image le masque seuillé et la raison du rejet (`0 quads`, `mal remplis` ou `REJETE` avec l'erreur de reprojection). C'est l'outil pour voir laquelle des trois hypothèses casse sur une vidéo donnée.

---

## Journal des séances

### Séance 1 — Détection de la feuille
- **Fait :** passage en niveaux de gris, seuil fixe, détection de la plus grande région claire avec MSER, puis extraction des 4 coins (enveloppe convexe et `approxPolyDP`).
- **Problèmes :** le seuil fixe (180) dépend de l'éclairage, et MSER est lent (~60 ms par image).

### Séance 2 — Pose de la feuille
- **Fait :** chargement de `K` et de la distorsion depuis `input_camera.yaml`, ordre des coins, `solvePnP`, dessin d'un cube et du repère avec `projectPoints`.
- **Choix :** repère de la feuille en mm, origine au centre, Z vers la caméra ; méthode `SOLVEPNP_IPPE`, adaptée aux cibles planes.
- **Notes du prof :** l'algo doit tourner en temps réel, être prêt pour un changement de caméra, travailler en sous-résolution, lisser les erreurs, et être testé sur plusieurs vidéos pour ne pas dépendre d'une seule.

### Séance 3 — Temps réel et outils de test
- **Fait :** nouveau suivi (`SheetTracker`) avec zone de recherche, sous-résolution, Otsu, `cornerSubPix`, ordre des coins cohérent dans le temps, validation par l'erreur de reprojection, lissage One-Euro et gestion des pertes. Le calcul passe de 60 ms à 2,5 ms par image.
- **Fait :** menu de choix des vidéos, lecture d'image fixe et de webcam, mode test `--bench`, générateur de vidéos de synthèse.
- **Choix :** labyrinthe modélisé directement en OpenGL, sans importer de fichier glTF. Il doit être généré automatiquement (plusieurs niveaux), et la même grille sert au dessin et aux collisions.

### Séance 4 — Tests sur données réelles
- **Fait :** premiers tests sur la vidéo et l'image du prof, et sur une vidéo à nous, volontairement difficile (surface en mousse, téléphone en portrait, filmé à la main).
- **Corrigé :** la matrice `K` est maintenant mise à l'échelle avec le même facteur en x et en y quand le format de la vidéo diffère du fichier caméra (4:3, portrait). Avant, `K` était déformée et toutes les poses étaient rejetées. `input.jpg` passe de 0 % à 100 %.
- **Ajouté :** option `--dump` pour enregistrer des images de diagnostic. Le seuil de rejet s'adapte maintenant à la résolution.
- **Constat :** l'algorithme est encore trop lié à la feuille A4 blanche. Objectif suivant : toute surface rectangulaire, même mal filmée (voir [Tests de robustesse](#tests-de-robustesse)).

## Prochaines étapes

- **Partie AR :**
  - caméra OpenGL à partir de `K` et de la pose, avec la vidéo en fond ;
  - enregistrement de la sortie en vidéo ;
  - calibration de notre caméra avec `calibrateCamera` ;
  - tests sur nos propres vidéos et sur la webcam ;
  - suivi de n'importe quelle surface rectangulaire (dimensions en paramètre, détection indépendante de la couleur) ;
  - comprendre pourquoi `input.mp4` n'est suivi que sur 24 % des images.
- **Partie VR :**
  - labyrinthe 3D posé sur la feuille ;
  - sphère texturée en fond ;
  - éclairage (Phong ou Blinn-Phong) et ombres ;
  - bascule entre mode AR et mode VR.
- **Partie MR :**
  - bille accélérée par l'inclinaison (gravité exprimée dans le repère de la feuille) ;
  - collisions et rebonds ;
  - physique à pas fixe, découplée de la vidéo ;
  - génération des labyrinthes (DFS avec une graine), niveaux, HUD.
