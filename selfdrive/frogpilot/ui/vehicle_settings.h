#pragma once

#include <QStringList>

#include "selfdrive/ui/qt/offroad/settings.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"

class FrogPilotVehiclesPanel : public ListWidget {
  Q_OBJECT

public:
  explicit FrogPilotVehiclesPanel(SettingsWindow *parent);

private:
  void setModels();
  void setToggles();

  std::vector<ToggleControl*> currentToggles;

  ButtonControl *selectMakeButton;
  ButtonControl *selectModelButton;

  ToggleControl *lockDoorsToggle;
  ToggleControl *sngHackToggle;
  ToggleControl *tss2TuneToggle;

  ToggleControl *evTableToggle;
  ToggleControl *lowerVoltToggle;
  ToggleControl *longPitch;

  QString brandSelection;
  QStringList models;

  Params params;
};
