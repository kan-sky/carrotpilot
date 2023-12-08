#pragma once

#include <QStringList>

#include "selfdrive/ui/qt/offroad/settings.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"

class FrogPilotVehiclesPanel : public QFrame {
  Q_OBJECT

public:
  explicit FrogPilotVehiclesPanel(QWidget *parent = 0);

private:
  void setModels();
  void updateToggles();

  ButtonControl *selectMakeButton;
  ButtonControl *selectModelButton;

  ToggleControl *lockDoorsToggle;
  ToggleControl *sngHackToggle;
  ToggleControl *tss2TuneToggle;

  ToggleControl *evTableToggle;
  ToggleControl *lowerVoltToggle;

  QString brandSelection;
  QStringList models;

  ListWidget *carSelectionList;
  ListWidget *gmToggles;
  ListWidget *toyotaToggles;
  QStackedLayout *mainLayout;
  QWidget *vehiclesWidget;

  Params params;
};
