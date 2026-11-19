#ifndef AI_CAMERA_H
#define AI_CAMERA_H

#include <Arduino.h>

// ---------------------------------------------------------------
// EDIT THESE to match the categories you trained your model on.
// The ORDER must match the category list from training
// (0 = first category, 1 = second, etc.).
// ---------------------------------------------------------------
enum Material {
  MATERIAL_UNKNOWN = -1,
  MATERIAL_FOAM  = 0,   
  MATERIAL_PLASTIC = 1,
};

class AiCamera {
public:
  // modelIndex    = the model index you used on the model uploader page
  // numCategories = how many categories you trained
  // samples       = camera readings taken per decision
  AiCamera(int modelIndex, int numCategories, int samples = 7);

  // Takes several predictions and returns the most common one.
  // getPrediction is a small function from the .ino that calls
  // Enes100.MLGetPrediction(index) and returns its result.
  Material detect(int (*getPrediction)(int));

  // Readable name for printing
  const char* name(Material m);

private:
  int _modelIndex;
  int _numCategories;
  int _samples;
};

#endif