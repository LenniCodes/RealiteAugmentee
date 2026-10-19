#ifndef TRACKER_HPP
#define TRACKER_HPP

#include <opencv2/core.hpp>
#include <array>
#include <string>
#include <vector>

#include "pose.hpp"

/// Filtre One-Euro (Casiez et al., 2012) : lisse fort quand le signal bouge peu
/// (supprime le tremblement) et peu quand il bouge vite (pas de retard).
class OneEuroFilter {
 public:
  OneEuroFilter(double minCutoff = 1.0, double beta = 0.01, double dCutoff = 1.0)
      : minCutoff_(minCutoff), beta_(beta), dCutoff_(dCutoff) {}
  double filter(double x, double dt);
  void reset() { init_ = false; }

 private:
  static double alpha(double cutoff, double dt);
  double minCutoff_, beta_, dCutoff_;
  double xPrev_ = 0, dxPrev_ = 0;
  bool init_ = false;
};

/// Réglages du suivi. Les valeurs par défaut conviennent pour du 720p/1080p.
struct TrackerParams {
  int procWidth = 480;          ///< largeur max (px) de l'image traitée par la détection
  double roiMargin = 0.35;      ///< marge autour du quad précédent (fraction de sa taille)
  double minAreaFrac = 0.01;    ///< aire min du quad / aire de l'image traitée
  double maxReprojErr = 4.0;    ///< erreur de reprojection max (px, pleine résolution)
  int maxCoastFrames = 5;       ///< nb d'images où l'on garde la dernière pose si la détection échoue
  int openKernel = 5;           ///< taille (px, image réduite) de l'ouverture morphologique
  bool subPix = true;           ///< raffinement sub-pixel des coins
  double filterMinCutoff = 1.0; ///< One-Euro : lissage au repos (plus petit = plus lisse)
  double filterBeta = 0.1;      ///< One-Euro : réactivité au mouvement (plus grand = moins de retard)
};

enum class TrackState { Lost, Tracking, Coasting };

/// Détection + suivi temps réel de la feuille A4 et estimation de sa pose.
///
/// Pipeline par image :
///   1. zone de recherche = autour de la feuille précédente (ou toute l'image si perdue)
///   2. sous-résolution, flou, seuil d'Otsu, contours, approximation en quadrilatère
///   3. coins remis en pleine résolution + cornerSubPix
///   4. coins ordonnés de façon cohérente avec l'image précédente (pas de saut de 90/180°)
///   5. validation par solvePnP (erreur de reprojection), lissage One-Euro des coins
class SheetTracker {
 public:
  SheetTracker(const CameraParams& cam, const TrackerParams& p = TrackerParams());

  /// Traite une image. `dt` = temps écoulé depuis l'image précédente (s).
  /// Renvoie true si une pose est disponible (Tracking ou Coasting).
  bool process(const cv::Mat& frameBGR, double dt);

  TrackState state() const { return state_; }
  bool hasPose() const { return state_ != TrackState::Lost; }
  const std::vector<cv::Point2f>& corners() const { return smooth_; }  ///< coins lissés
  const std::vector<cv::Point2f>& rawCorners() const { return raw_; }  ///< coins bruts
  const cv::Mat& rvec() const { return rvec_; }
  const cv::Mat& tvec() const { return tvec_; }
  double reprojError() const { return reprojErr_; }
  const cv::Rect& searchRect() const { return search_; }    ///< zone de recherche (debug)
  const cv::Mat& debugMask() const { return mask_; }        ///< image seuillée (debug)
  const std::string& debugInfo() const { return info_; }    ///< pourquoi la détection a réussi/échoué
  void reset();

 private:
  bool detect(const cv::Mat& gray, std::vector<cv::Point2f>& out);
  std::vector<cv::Point2f> matchOrder(const std::vector<cv::Point2f>& ordered) const;

  CameraParams cam_;
  TrackerParams p_;
  TrackState state_ = TrackState::Lost;
  int coast_ = 0;
  std::vector<cv::Point2f> raw_, smooth_;
  std::array<OneEuroFilter, 8> filters_;
  cv::Mat rvec_, tvec_;
  double reprojErr_ = 0;
  cv::Rect search_;
  cv::Mat gray_, mask_;
  std::string info_;
};

#endif
