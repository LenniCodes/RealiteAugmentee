#ifndef MENU_HPP
#define MENU_HPP

#include <opencv2/core.hpp>
#include <string>
#include <vector>

/// Une entrée du menu : un fichier vidéo/image, ou une webcam.
struct MediaEntry {
  std::string path;   ///< chemin du fichier, ou index de webcam ("0", "1"...)
  std::string label;  ///< nom affiché
  bool isCamera = false;
  // Infos lues à l'ouverture (pour l'affichage)
  cv::Size size;
  double fps = 0;
  int frames = 0;
  cv::Mat thumb;
};

/// Liste les vidéos et images d'un dossier (triées), puis ajoute la webcam 0.
std::vector<MediaEntry> scanMedia(const std::string& folder, bool addWebcam = true);

/// Affiche le menu et attend un choix.
/// Renvoie l'indice choisi, -1 pour quitter, -2 pour relancer le scan du dossier.
int runMenu(std::vector<MediaEntry>& entries, const std::string& folder, int selected = 0);

/// Lecture des codes de touches flèches (valeurs différentes selon l'OS / backend).
bool isKeyUp(int k);
bool isKeyDown(int k);
bool isKeyEnter(int k);
bool isKeyEscape(int k);

#endif
