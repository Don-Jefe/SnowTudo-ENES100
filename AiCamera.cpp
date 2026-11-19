#include "AiCamera.h"
// Note: do NOT include Enes100.h here. It would be defined twice
// (once here, once in the .ino) and cause a "multiple definition" error.

AiCamera::AiCamera(int modelIndex, int numCategories, int samples)
  : _modelIndex(modelIndex), _numCategories(numCategories), _samples(samples) {}

Material AiCamera::detect(int (*getPrediction)(int)) {
  int votes[8] = {0};                       // supports up to 8 categories
  int maxCats = min(_numCategories, 8);

  for (int i = 0; i < _samples; i++) {
    int p = getPrediction(_modelIndex);
    if (p >= 0 && p < maxCats) {
      votes[p]++;
    }
    delay(150);                             // let the camera grab a fresh frame
  }

  int best = -1;
  int bestVotes = 0;
  for (int c = 0; c < maxCats; c++) {
    if (votes[c] > bestVotes) {
      bestVotes = votes[c];
      best = c;
    }
  }

  // Require a clear majority, otherwise report unknown
  if (best == -1 || bestVotes <= _samples / 2) {
    return MATERIAL_UNKNOWN;
  }
  return (Material)best;
}

const char* AiCamera::name(Material m) {
  switch (m) {
    case MATERIAL_FOAM:    return "Foam";
    case MATERIAL_PLASTIC: return "Plastic";
    default:               return "Unknown";
  }
}