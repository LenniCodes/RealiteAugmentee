# ReVA — Réalité augmentée avec OpenCV

Projet du cours de réalité augmentée / virtuelle / mixte (Polytech Marseille, 5A).
Sujet : <https://mavromatis.org/ravm/index.html>

Le projet se fait en trois parties : **AR** (suivi d'une feuille A4 et incrustation 3D), **VR** (labyrinthe et environnement virtuel) et **MR** (jeu de bille piloté par l'inclinaison de la feuille).
Ce README décrit où on en est sur la **partie AR**.

## Ce qui fonctionne

1. **Détection de la feuille A4** (`src/findPaperSheet.cpp`)
   - L'image est passée en niveaux de gris, puis seuillée (`threshold`) pour garder les zones claires.
   - On cherche les régions stables avec **MSER** et on garde la plus grande : c'est la feuille.
2. **Extraction des 4 coins** (`src/main.cpp`)
   - On prend le contour de la feuille, son enveloppe convexe, puis on l'approxime par un polygone (`approxPolyDP`) jusqu'à obtenir 4 sommets.
   - La position des coins est ensuite affinée au sous-pixel avec `cornerSubPix`.
3. **Ordonnancement des coins** (`orderCorners`, dans `src/pose.cpp`)
   - Les coins sont triés dans le sens horaire, et l'arête 0→1 est toujours un grand côté de la feuille.
   - Cette étape est indispensable : chaque coin de l'image doit correspondre au bon coin du modèle 3D.
4. **Paramètres de la caméra** (`loadCameraParams`)
   - On lit la matrice intrinsèque `K` et les coefficients de distorsion dans `input_camera.yaml` (fichier fourni).
   - Si la vidéo n'a pas la résolution prévue par le fichier (1280×720), `K` est mis à l'échelle automatiquement.
5. **Estimation de la pose** (`estimatePose`)
   - La feuille est modélisée par ses 4 coins en 3D : A4 de 297 × 210 mm, origine au centre, Z qui sort de la feuille.
   - `cv::solvePnP` (méthode `SOLVEPNP_IPPE`, faite pour les cibles planes) donne la rotation `rvec` et la translation `tvec` de la feuille par rapport à la caméra, en mm.
   - On calcule aussi l'erreur de reprojection pour vérifier la qualité de la pose.
6. **Incrustation 3D** (`drawCube`)
   - Un cube de 10 cm posé sur la feuille et le repère XYZ sont projetés dans l'image avec `projectPoints` et `drawFrameAxes`.

> Remarque : `solvePnP` ne calcule pas la distorsion. Il prend `K` et les coefficients de distorsion **en entrée**.
> Pour calibrer notre propre caméra, il faudra utiliser `cv::calibrateCamera` avec un damier.

## Structure

```
include/
  utils.hpp          toGrayScale, findPaperSheet
  pose.hpp           CameraParams, orderCorners, estimatePose, drawCube
src/
  main.cpp           boucle vidéo : détection -> coins -> pose -> affichage
  findPaperSheet.cpp détection de la feuille (seuil + MSER)
  toGrayScale.cpp
  pose.cpp           paramètres caméra, ordre des coins, solvePnP, dessin du cube
input_camera.yaml    paramètres intrinsèques fournis (K, distorsion)
```

## Compiler et lancer

Dépendances : CMake, un compilateur C++ et OpenCV 4. Le code est testé avec OpenCV 4.5 et 4.6.

```bash
# Ubuntu / Debian
sudo apt install build-essential cmake ninja-build libopencv-dev
# ou avec Nix
nix --extra-experimental-features 'nix-command flakes' develop
```

```bash
# vidéo et image de test fournies par le prof (non versionnées)
wget https://mavromatis.org/ravm/input.mp4 https://mavromatis.org/ravm/input.jpg

cmake -S . -B build -G Ninja
cmake --build build
cd build && ./CoursOpenCv ../input.mp4 ../input_camera.yaml
```

Usage : `./CoursOpenCv [video] [camera.yaml]`

Touches : `Échap` ou `q` pour quitter, `Espace` pour mettre en pause.

À l'écran : les coins de la feuille sont numérotés de 0 à 3, le cube et le repère sont dessinés (X rouge, Y vert, Z bleu), et en haut s'affichent la distance caméra–feuille en mm et l'erreur de reprojection en pixels.

## Limites connues et prochaines étapes

- **Ambiguïté de 180°** : la feuille est symétrique, donc la pose peut basculer d'une demi-rotation entre deux images. Ça ne gêne pas pour le cube, mais il faudra la lever pour le labyrinthe. Deux pistes : garder la rotation la plus proche de celle de l'image précédente, ou dessiner un repère sur la feuille.
- **Stabilité** : la pose tremble un peu. Il faudra lisser `rvec`/`tvec` d'une image à l'autre et gérer les images où la feuille n'est pas détectée.
- **Robustesse de la détection** : le seuil fixe (180) dépend de l'éclairage. Il faudra tester un seuil adaptatif ou la méthode d'Otsu.
- **Calibration** : calibrer notre propre caméra avec `calibrateCamera` (damier).
- **Tests** : image fixe (`input.jpg`), nos propres vidéos, flux webcam en direct.
- **Rendu** : passer l'affichage 3D en OpenGL (partie VR), et enregistrer la sortie en vidéo.
