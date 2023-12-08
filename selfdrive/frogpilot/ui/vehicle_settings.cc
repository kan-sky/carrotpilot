#include <QDir>
#include <QRegularExpression>
#include <QTextStream>

#include "selfdrive/frogpilot/ui/vehicle_settings.h"

QStringList getCarNames(const QString &dirPath, const QString &carBrand) {
  QMap<QString, QString> brandMap;
  brandMap["acura"] = "honda";
  brandMap["audi"] = "volkswagen";
  brandMap["buick"] = "gm";
  brandMap["cadillac"] = "gm";
  brandMap["chevrolet"] = "gm";
  brandMap["chrysler"] = "chrysler";
  brandMap["dodge"] = "chrysler";
  brandMap["ford"] = "ford";
  brandMap["gm"] = "gm";
  brandMap["gmc"] = "gm";
  brandMap["genesis"] = "hyundai";
  brandMap["honda"] = "honda";
  brandMap["hyundai"] = "hyundai";
  brandMap["infiniti"] = "nissan";
  brandMap["jeep"] = "chrysler";
  brandMap["kia"] = "hyundai";
  brandMap["lexus"] = "toyota";
  brandMap["lincoln"] = "ford";
  brandMap["man"] = "volkswagen";
  brandMap["mazda"] = "mazda";
  brandMap["mercedes"] = "gm";
  brandMap["nissan"] = "nissan";
  brandMap["ram"] = "chrysler";
  brandMap["seat"] = "volkswagen";
  brandMap["subaru"] = "subaru";
  brandMap["tesla"] = "tesla";
  brandMap["toyota"] = "toyota";
  brandMap["volkswagen"] = "volkswagen";
  brandMap["volvo"] = "gm";
  brandMap["skoda"] = "volkswagen";

  QStringList names;
  QDir dir(dirPath);
  QString lowerCaseCarBrand = carBrand.toLower();

  QString targetFolder = brandMap.value(lowerCaseCarBrand, lowerCaseCarBrand);

  foreach (const QString &folder, dir.entryList(QDir::Dirs | QDir::NoDotAndDotDot)) {
    if (folder.toLower() == targetFolder) {
      QFile file(dirPath + "/" + folder + "/values.py");
      if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QRegularExpression regex("class CAR\\(StrEnum\\):([\\s\\S]*?)(?=^\\w)", QRegularExpression::MultilineOption);
        QRegularExpressionMatch match = regex.match(QTextStream(&file).readAll());
        file.close();

        if (match.hasMatch()) {
          QRegularExpression nameRegex("=\\s*\"([^\"]+)\"");
          QRegularExpressionMatchIterator it = nameRegex.globalMatch(match.captured(1));
          while (it.hasNext()) {
            names << it.next().captured(1);
          }
        }
      }
    }
  }

  std::sort(names.begin(), names.end());
  return names;
}

FrogPilotVehiclesPanel::FrogPilotVehiclesPanel(QWidget *parent) : QFrame(parent) {
  mainLayout = new QStackedLayout(this);

  vehiclesWidget = new QWidget();
  QVBoxLayout *vehiclesLayout = new QVBoxLayout(vehiclesWidget);
  vehiclesLayout->setMargin(40);

  carSelectionList = new ListWidget(this);

  selectMakeButton = new ButtonControl(tr("Select Make"), tr("SELECT"));
  brandSelection = QString::fromStdString(params.get("CarBrand"));
  connect(selectMakeButton, &ButtonControl::clicked, [this]() {
    std::string current = params.get("CarBrand");
    QStringList makes = {
      "Acura", "Audi", "BMW", "Buick", "Cadillac", "Chevrolet", "Chrysler", "Dodge", "Ford", "GM", "GMC", 
      "Genesis", "Honda", "Hyundai", "Infiniti", "Jeep", "Kia", "Lexus", "Lincoln", "MAN", "Mazda", 
      "Mercedes", "Nissan", "Ram", "SEAT", "Subaru", "Tesla", "Toyota", "Volkswagen", "Volvo", "?koda"
    };

    brandSelection = MultiOptionDialog::getSelection(tr("Select a Make"), makes, QString::fromStdString(current), this);
    if (!brandSelection.isEmpty()) {
      params.put("CarBrand", brandSelection.toStdString());
      selectMakeButton->setValue(brandSelection);
      setModels();
      updateToggles();
    }
  });
  carSelectionList->addItem(selectMakeButton);
  selectMakeButton->setValue(brandSelection);

  selectModelButton = new ButtonControl(tr("Select Model"), tr("SELECT"));
  connect(selectModelButton, &ButtonControl::clicked, [this]() {
    std::string current = params.get("CarModel");
    QString modelSelection = MultiOptionDialog::getSelection(tr("Select a Model"), models, QString::fromStdString(current), this);
    if (!modelSelection.isEmpty()) {
      params.put("CarModel", modelSelection.toStdString());
      selectModelButton->setValue(modelSelection);
    }
  });
  carSelectionList->addItem(selectModelButton);
  selectModelButton->setValue(QString::fromStdString(params.get("CarModel")));

  gmToggles = new ListWidget(this);

  const bool evTable = params.getBool("EVTable");
  evTableToggle = new ToggleControl(tr("EV Lookup Tables"), "Smoothens out the gas and brake controls for EV vehicles.", "", evTable);
  QObject::connect(evTableToggle, &ToggleControl::toggleFlipped, [=](bool state) {
    params.putBool("EVTable", state);
  });
  gmToggles->addItem(evTableToggle);

  const bool lowerVolt = params.getBool("LowerVolt");
  lowerVoltToggle = new ToggleControl(tr("Lower Volt Enable Speed"), "Lowers the Volt's minimum enable speed in order to enable openpilot at any speed.", "", lowerVolt);
  QObject::connect(lowerVoltToggle, &ToggleControl::toggleFlipped, [=](bool state) {
    params.putBool("LowerVolt", state);
  });
  gmToggles->addItem(lowerVoltToggle);

  toyotaToggles = new ListWidget(this);

  const bool lockDoors = params.getBool("LockDoors");
  lockDoorsToggle = new ToggleControl(tr("Lock Doors In Drive"), "Automatically locks the doors when in drive and unlocks when in park.", "", lockDoors);
  QObject::connect(lockDoorsToggle, &ToggleControl::toggleFlipped, [=](bool state) {
    params.putBool("LockDoors", state);
  });
  toyotaToggles->addItem(lockDoorsToggle);

  const bool sngHack = params.getBool("SNGHack");
  sngHackToggle = new ToggleControl(tr("Stop and Go Hack"), "Enable the 'Stop and Go' hack for vehicles without stock stop and go functionality.", "", sngHack);
  QObject::connect(sngHackToggle, &ToggleControl::toggleFlipped, [=](bool state) {
    params.putBool("SNGHack", state);
  });
  toyotaToggles->addItem(sngHackToggle);

  const bool tss2Tune = params.getBool("TSS2Tune");
  tss2TuneToggle = new ToggleControl(tr("TSS2 Tune"), "Tuning profile based on the tuning profile from DragonPilot for TSS2 vehicles.", "", tss2Tune);
  QObject::connect(tss2TuneToggle, &ToggleControl::toggleFlipped, [=](bool state) {
    params.putBool("TSS2Tune", state);
  });
  toyotaToggles->addItem(tss2TuneToggle);

  vehiclesLayout->addWidget(new ScrollView(carSelectionList, vehiclesWidget));
  vehiclesLayout->addWidget(new ScrollView(gmToggles, this));
  vehiclesLayout->addWidget(new ScrollView(toyotaToggles, this));
  vehiclesWidget->setLayout(vehiclesLayout);
  mainLayout->addWidget(vehiclesWidget);
  mainLayout->setCurrentWidget(vehiclesWidget);

  setModels();
  updateToggles();
}

void FrogPilotVehiclesPanel::setModels() {
  std::string carBrand = params.get("CarBrand");
  QString dirPath = "/data/openpilot/selfdrive/car";
  models = getCarNames(dirPath, QString::fromStdString(carBrand));
}

void FrogPilotVehiclesPanel::updateToggles() {
  const bool GM = (brandSelection == "Buick" || brandSelection == "Cadillac" || brandSelection == "Chevrolet" || brandSelection == "GM" || brandSelection == "GMC");
  const bool Toyota = (brandSelection == "Lexus" || brandSelection == "Toyota");

  gmToggles->setVisible(GM);
  toyotaToggles->setVisible(Toyota);

  mainLayout->removeWidget(gmToggles);
  mainLayout->removeWidget(toyotaToggles);

  if (GM) {
    mainLayout->addWidget(gmToggles);
    mainLayout->setCurrentWidget(gmToggles);
  } else if (Toyota) {
    mainLayout->addWidget(toyotaToggles);
    mainLayout->setCurrentWidget(toyotaToggles);
  }
}
