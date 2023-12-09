#pragma once

#include <QStringList>

#include "selfdrive/ui/qt/offroad/settings.h"

class FrogPilotVehiclesPanel : public ListWidget {
  Q_OBJECT

public:
  explicit FrogPilotVehiclesPanel(SettingsWindow *parent);

private:
  void setModels();
  void setToggles();

  ButtonControl *selectMakeButton;
  ButtonControl *selectModelButton;

  ToggleControl *lockDoorsToggle=0;
  ToggleControl *sngHackToggle=0;
  ToggleControl *tss2TuneToggle=0;

  ToggleControl *evTableToggle=0;
  ToggleControl *longPitchToggle=0;
  ToggleControl *lowerVoltToggle=0;

  QString brandSelection;
  QStringList models;

  Params params;
  Params paramsMemory{"/dev/shm/params"};
};
