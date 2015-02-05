/********************************************************************************
** Form generated from reading UI file 'plugin.ui'
**
** Created: Tue Sep 30 21:20:45 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PLUGIN_H
#define UI_PLUGIN_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDockWidget>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QScrollArea>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QTabWidget>
#include <QtGui/QTreeWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_plugin
{
public:
    QWidget *dockWidgetContents;
    QHBoxLayout *horizontalLayout;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QVBoxLayout *verticalLayout;
    QVBoxLayout *verticalLayout_2;
    QTreeWidget *treeWidget;
    QHBoxLayout *horizontalLayout_12;
    QTabWidget *tabWidget;
    QWidget *tab;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_11;
    QLabel *label_8;
    QSpacerItem *horizontalSpacer;
    QLineEdit *lineModelName;
    QHBoxLayout *horizontalLayout_7;
    QLabel *label_6;
    QSpacerItem *horizontalSpacer_2;
    QSpinBox *spinRefinemantSteps;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_4;
    QSpacerItem *horizontalSpacer_3;
    QComboBox *comboBoxSelectSensor;
    QSpacerItem *verticalSpacer_2;
    QPushButton *btnCreateModel;
    QWidget *tab_3;
    QVBoxLayout *verticalLayout_5;
    QHBoxLayout *horizontalLayout_13;
    QLabel *label_logging_3;
    QSpacerItem *horizontalSpacer_7;
    QCheckBox *checkBoxLogPose;
    QHBoxLayout *horizontalLayout_8;
    QLabel *label_7;
    QSpacerItem *horizontalSpacer_4;
    QComboBox *comboBoPEMethod;
    QHBoxLayout *horizontalLayout_9;
    QLabel *label_9;
    QSpacerItem *horizontalSpacer_5;
    QSpinBox *spinBoxSceneInstance;
    QHBoxLayout *horizontalLayout_10;
    QLabel *label_10;
    QSpacerItem *horizontalSpacer_6;
    QLineEdit *lineSceneName;
    QSpacerItem *verticalSpacer;
    QPushButton *btnEstimatePose;
    QWidget *tab_4;
    QVBoxLayout *verticalLayout_6;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_5;
    QSpacerItem *horizontalSpacer_8;
    QSpinBox *spinBoxSamples;
    QHBoxLayout *horizontalLayout_15;
    QLabel *label_12;
    QSpacerItem *horizontalSpacer_9;
    QSpinBox *spinBoxTarget;
    QHBoxLayout *horizontalLayout_14;
    QLabel *label_11;
    QSpacerItem *horizontalSpacer_10;
    QComboBox *comboBoxGripperGraspGeneration;
    QCheckBox *checkBoxPertubation;
    QHBoxLayout *horizontalLayout_16;
    QLabel *label_13;
    QSpacerItem *horizontalSpacer_11;
    QSpinBox *spinBoxPertubations;
    QHBoxLayout *horizontalLayout_17;
    QLabel *labelSigmaPos;
    QLineEdit *lineEditSigmaPos;
    QLabel *labelSigmaR;
    QLineEdit *lineEditsigmaRot;
    QSpacerItem *verticalSpacer_5;
    QHBoxLayout *horizontalLayout_19;
    QPushButton *btnLoadGraspScene;
    QPushButton *btnRotateObject;
    QPushButton *btnGenerateGraspTable;
    QWidget *tab_2;
    QVBoxLayout *verticalLayout_3;
    QLabel *label;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_2;
    QComboBox *comboBoxRobot;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_3;
    QComboBox *comboBoxGripper;
    QHBoxLayout *horizontalLayout_18;
    QCheckBox *checkBoxSimulate;
    QPushButton *btnGrasp;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *btnTest;
    QCheckBox *checkBoxLogGrasp;
    QCheckBox *checkBoxLiveUpdate;
    QSpacerItem *verticalSpacer_3;

    void setupUi(QDockWidget *plugin)
    {
        if (plugin->objectName().isEmpty())
            plugin->setObjectName(QString::fromUtf8("plugin"));
        plugin->resize(1015, 346);
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(plugin->sizePolicy().hasHeightForWidth());
        plugin->setSizePolicy(sizePolicy);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        horizontalLayout = new QHBoxLayout(dockWidgetContents);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        scrollArea = new QScrollArea(dockWidgetContents);
        scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
        sizePolicy.setHeightForWidth(scrollArea->sizePolicy().hasHeightForWidth());
        scrollArea->setSizePolicy(sizePolicy);
        scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 494, 301));
        verticalLayout = new QVBoxLayout(scrollAreaWidgetContents);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        treeWidget = new QTreeWidget(scrollAreaWidgetContents);
        QTreeWidgetItem *__qtreewidgetitem = new QTreeWidgetItem();
        __qtreewidgetitem->setText(0, QString::fromUtf8("1"));
        treeWidget->setHeaderItem(__qtreewidgetitem);
        treeWidget->setObjectName(QString::fromUtf8("treeWidget"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(treeWidget->sizePolicy().hasHeightForWidth());
        treeWidget->setSizePolicy(sizePolicy1);
        treeWidget->setMinimumSize(QSize(292, 222));

        verticalLayout_2->addWidget(treeWidget);


        verticalLayout->addLayout(verticalLayout_2);

        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));

        verticalLayout->addLayout(horizontalLayout_12);

        scrollArea->setWidget(scrollAreaWidgetContents);

        horizontalLayout->addWidget(scrollArea);

        tabWidget = new QTabWidget(dockWidgetContents);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setTabShape(QTabWidget::Rounded);
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        verticalLayout_4 = new QVBoxLayout(tab);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        label_8 = new QLabel(tab);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        label_8->setFont(font);

        horizontalLayout_11->addWidget(label_8);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_11->addItem(horizontalSpacer);

        lineModelName = new QLineEdit(tab);
        lineModelName->setObjectName(QString::fromUtf8("lineModelName"));

        horizontalLayout_11->addWidget(lineModelName);


        verticalLayout_4->addLayout(horizontalLayout_11);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        label_6 = new QLabel(tab);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setFont(font);

        horizontalLayout_7->addWidget(label_6);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_7->addItem(horizontalSpacer_2);

        spinRefinemantSteps = new QSpinBox(tab);
        spinRefinemantSteps->setObjectName(QString::fromUtf8("spinRefinemantSteps"));
        spinRefinemantSteps->setMinimum(2);
        spinRefinemantSteps->setMaximum(10);

        horizontalLayout_7->addWidget(spinRefinemantSteps);


        verticalLayout_4->addLayout(horizontalLayout_7);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label_4 = new QLabel(tab);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setFont(font);

        horizontalLayout_5->addWidget(label_4);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer_3);

        comboBoxSelectSensor = new QComboBox(tab);
        comboBoxSelectSensor->setObjectName(QString::fromUtf8("comboBoxSelectSensor"));

        horizontalLayout_5->addWidget(comboBoxSelectSensor);


        verticalLayout_4->addLayout(horizontalLayout_5);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_4->addItem(verticalSpacer_2);

        btnCreateModel = new QPushButton(tab);
        btnCreateModel->setObjectName(QString::fromUtf8("btnCreateModel"));

        verticalLayout_4->addWidget(btnCreateModel);

        tabWidget->addTab(tab, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        verticalLayout_5 = new QVBoxLayout(tab_3);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        horizontalLayout_13 = new QHBoxLayout();
        horizontalLayout_13->setObjectName(QString::fromUtf8("horizontalLayout_13"));
        label_logging_3 = new QLabel(tab_3);
        label_logging_3->setObjectName(QString::fromUtf8("label_logging_3"));
        label_logging_3->setFont(font);

        horizontalLayout_13->addWidget(label_logging_3);

        horizontalSpacer_7 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_13->addItem(horizontalSpacer_7);

        checkBoxLogPose = new QCheckBox(tab_3);
        checkBoxLogPose->setObjectName(QString::fromUtf8("checkBoxLogPose"));
        checkBoxLogPose->setChecked(true);

        horizontalLayout_13->addWidget(checkBoxLogPose);


        verticalLayout_5->addLayout(horizontalLayout_13);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        label_7 = new QLabel(tab_3);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setFont(font);

        horizontalLayout_8->addWidget(label_7);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_8->addItem(horizontalSpacer_4);

        comboBoPEMethod = new QComboBox(tab_3);
        comboBoPEMethod->setObjectName(QString::fromUtf8("comboBoPEMethod"));

        horizontalLayout_8->addWidget(comboBoPEMethod);


        verticalLayout_5->addLayout(horizontalLayout_8);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        label_9 = new QLabel(tab_3);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setFont(font);

        horizontalLayout_9->addWidget(label_9);

        horizontalSpacer_5 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_9->addItem(horizontalSpacer_5);

        spinBoxSceneInstance = new QSpinBox(tab_3);
        spinBoxSceneInstance->setObjectName(QString::fromUtf8("spinBoxSceneInstance"));
        spinBoxSceneInstance->setValue(1);

        horizontalLayout_9->addWidget(spinBoxSceneInstance);


        verticalLayout_5->addLayout(horizontalLayout_9);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        label_10 = new QLabel(tab_3);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setFont(font);

        horizontalLayout_10->addWidget(label_10);

        horizontalSpacer_6 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_10->addItem(horizontalSpacer_6);

        lineSceneName = new QLineEdit(tab_3);
        lineSceneName->setObjectName(QString::fromUtf8("lineSceneName"));

        horizontalLayout_10->addWidget(lineSceneName);


        verticalLayout_5->addLayout(horizontalLayout_10);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_5->addItem(verticalSpacer);

        btnEstimatePose = new QPushButton(tab_3);
        btnEstimatePose->setObjectName(QString::fromUtf8("btnEstimatePose"));

        verticalLayout_5->addWidget(btnEstimatePose);

        tabWidget->addTab(tab_3, QString());
        tab_4 = new QWidget();
        tab_4->setObjectName(QString::fromUtf8("tab_4"));
        verticalLayout_6 = new QVBoxLayout(tab_4);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        label_5 = new QLabel(tab_4);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setFont(font);

        horizontalLayout_6->addWidget(label_5);

        horizontalSpacer_8 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_6->addItem(horizontalSpacer_8);

        spinBoxSamples = new QSpinBox(tab_4);
        spinBoxSamples->setObjectName(QString::fromUtf8("spinBoxSamples"));
        spinBoxSamples->setMaximum(10000000);
        spinBoxSamples->setValue(100000);

        horizontalLayout_6->addWidget(spinBoxSamples);


        verticalLayout_6->addLayout(horizontalLayout_6);

        horizontalLayout_15 = new QHBoxLayout();
        horizontalLayout_15->setObjectName(QString::fromUtf8("horizontalLayout_15"));
        label_12 = new QLabel(tab_4);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        label_12->setFont(font);

        horizontalLayout_15->addWidget(label_12);

        horizontalSpacer_9 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_15->addItem(horizontalSpacer_9);

        spinBoxTarget = new QSpinBox(tab_4);
        spinBoxTarget->setObjectName(QString::fromUtf8("spinBoxTarget"));
        spinBoxTarget->setMaximum(10000);
        spinBoxTarget->setValue(1000);

        horizontalLayout_15->addWidget(spinBoxTarget);


        verticalLayout_6->addLayout(horizontalLayout_15);

        horizontalLayout_14 = new QHBoxLayout();
        horizontalLayout_14->setObjectName(QString::fromUtf8("horizontalLayout_14"));
        label_11 = new QLabel(tab_4);
        label_11->setObjectName(QString::fromUtf8("label_11"));
        label_11->setFont(font);

        horizontalLayout_14->addWidget(label_11);

        horizontalSpacer_10 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_14->addItem(horizontalSpacer_10);

        comboBoxGripperGraspGeneration = new QComboBox(tab_4);
        comboBoxGripperGraspGeneration->setObjectName(QString::fromUtf8("comboBoxGripperGraspGeneration"));

        horizontalLayout_14->addWidget(comboBoxGripperGraspGeneration);


        verticalLayout_6->addLayout(horizontalLayout_14);

        checkBoxPertubation = new QCheckBox(tab_4);
        checkBoxPertubation->setObjectName(QString::fromUtf8("checkBoxPertubation"));
        checkBoxPertubation->setChecked(true);

        verticalLayout_6->addWidget(checkBoxPertubation);

        horizontalLayout_16 = new QHBoxLayout();
        horizontalLayout_16->setObjectName(QString::fromUtf8("horizontalLayout_16"));
        label_13 = new QLabel(tab_4);
        label_13->setObjectName(QString::fromUtf8("label_13"));
        label_13->setFont(font);

        horizontalLayout_16->addWidget(label_13);

        horizontalSpacer_11 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_16->addItem(horizontalSpacer_11);

        spinBoxPertubations = new QSpinBox(tab_4);
        spinBoxPertubations->setObjectName(QString::fromUtf8("spinBoxPertubations"));
        spinBoxPertubations->setMaximum(1000);
        spinBoxPertubations->setValue(100);

        horizontalLayout_16->addWidget(spinBoxPertubations);


        verticalLayout_6->addLayout(horizontalLayout_16);

        horizontalLayout_17 = new QHBoxLayout();
        horizontalLayout_17->setObjectName(QString::fromUtf8("horizontalLayout_17"));
        labelSigmaPos = new QLabel(tab_4);
        labelSigmaPos->setObjectName(QString::fromUtf8("labelSigmaPos"));
        labelSigmaPos->setFont(font);

        horizontalLayout_17->addWidget(labelSigmaPos);

        lineEditSigmaPos = new QLineEdit(tab_4);
        lineEditSigmaPos->setObjectName(QString::fromUtf8("lineEditSigmaPos"));

        horizontalLayout_17->addWidget(lineEditSigmaPos);

        labelSigmaR = new QLabel(tab_4);
        labelSigmaR->setObjectName(QString::fromUtf8("labelSigmaR"));
        labelSigmaR->setFont(font);

        horizontalLayout_17->addWidget(labelSigmaR);

        lineEditsigmaRot = new QLineEdit(tab_4);
        lineEditsigmaRot->setObjectName(QString::fromUtf8("lineEditsigmaRot"));

        horizontalLayout_17->addWidget(lineEditsigmaRot);


        verticalLayout_6->addLayout(horizontalLayout_17);

        verticalSpacer_5 = new QSpacerItem(20, 13, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_6->addItem(verticalSpacer_5);

        horizontalLayout_19 = new QHBoxLayout();
        horizontalLayout_19->setObjectName(QString::fromUtf8("horizontalLayout_19"));
        btnLoadGraspScene = new QPushButton(tab_4);
        btnLoadGraspScene->setObjectName(QString::fromUtf8("btnLoadGraspScene"));

        horizontalLayout_19->addWidget(btnLoadGraspScene);

        btnRotateObject = new QPushButton(tab_4);
        btnRotateObject->setObjectName(QString::fromUtf8("btnRotateObject"));

        horizontalLayout_19->addWidget(btnRotateObject);

        btnGenerateGraspTable = new QPushButton(tab_4);
        btnGenerateGraspTable->setObjectName(QString::fromUtf8("btnGenerateGraspTable"));

        horizontalLayout_19->addWidget(btnGenerateGraspTable);


        verticalLayout_6->addLayout(horizontalLayout_19);

        tabWidget->addTab(tab_4, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        verticalLayout_3 = new QVBoxLayout(tab_2);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        label = new QLabel(tab_2);
        label->setObjectName(QString::fromUtf8("label"));
        label->setFont(font);

        verticalLayout_3->addWidget(label);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_2 = new QLabel(tab_2);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setFont(font);

        horizontalLayout_2->addWidget(label_2);

        comboBoxRobot = new QComboBox(tab_2);
        comboBoxRobot->setObjectName(QString::fromUtf8("comboBoxRobot"));
        comboBoxRobot->setMinimumSize(QSize(50, 0));

        horizontalLayout_2->addWidget(comboBoxRobot);


        verticalLayout_3->addLayout(horizontalLayout_2);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_3 = new QLabel(tab_2);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setFont(font);

        horizontalLayout_4->addWidget(label_3);

        comboBoxGripper = new QComboBox(tab_2);
        comboBoxGripper->setObjectName(QString::fromUtf8("comboBoxGripper"));

        horizontalLayout_4->addWidget(comboBoxGripper);


        verticalLayout_3->addLayout(horizontalLayout_4);

        horizontalLayout_18 = new QHBoxLayout();
        horizontalLayout_18->setObjectName(QString::fromUtf8("horizontalLayout_18"));
        checkBoxSimulate = new QCheckBox(tab_2);
        checkBoxSimulate->setObjectName(QString::fromUtf8("checkBoxSimulate"));

        horizontalLayout_18->addWidget(checkBoxSimulate);

        btnGrasp = new QPushButton(tab_2);
        btnGrasp->setObjectName(QString::fromUtf8("btnGrasp"));

        horizontalLayout_18->addWidget(btnGrasp);


        verticalLayout_3->addLayout(horizontalLayout_18);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        btnTest = new QPushButton(tab_2);
        btnTest->setObjectName(QString::fromUtf8("btnTest"));

        horizontalLayout_3->addWidget(btnTest);

        checkBoxLogGrasp = new QCheckBox(tab_2);
        checkBoxLogGrasp->setObjectName(QString::fromUtf8("checkBoxLogGrasp"));
        checkBoxLogGrasp->setChecked(true);

        horizontalLayout_3->addWidget(checkBoxLogGrasp);


        verticalLayout_3->addLayout(horizontalLayout_3);

        checkBoxLiveUpdate = new QCheckBox(tab_2);
        checkBoxLiveUpdate->setObjectName(QString::fromUtf8("checkBoxLiveUpdate"));
        checkBoxLiveUpdate->setChecked(true);

        verticalLayout_3->addWidget(checkBoxLiveUpdate);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer_3);

        tabWidget->addTab(tab_2, QString());

        horizontalLayout->addWidget(tabWidget);

        plugin->setWidget(dockWidgetContents);

        retranslateUi(plugin);

        tabWidget->setCurrentIndex(2);


        QMetaObject::connectSlotsByName(plugin);
    } // setupUi

    void retranslateUi(QDockWidget *plugin)
    {
        plugin->setWindowTitle(QApplication::translate("plugin", "One Shot Learning Plugin", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("plugin", "Model name ", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("plugin", "Refinement steps", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("plugin", "Sensor", 0, QApplication::UnicodeUTF8));
        btnCreateModel->setText(QApplication::translate("plugin", "Create Model", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("plugin", "Create Model", 0, QApplication::UnicodeUTF8));
        label_logging_3->setText(QApplication::translate("plugin", "Logging", 0, QApplication::UnicodeUTF8));
        checkBoxLogPose->setText(QApplication::translate("plugin", "Log Pose Estimation Result", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("plugin", "Pose estimation method", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("plugin", "Scene Instances", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("plugin", "Scene Name", 0, QApplication::UnicodeUTF8));
        btnEstimatePose->setText(QApplication::translate("plugin", "Estimate Pose", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("plugin", "Estimate Pose", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("plugin", "Samples", 0, QApplication::UnicodeUTF8));
        label_12->setText(QApplication::translate("plugin", "Target pr. sample", 0, QApplication::UnicodeUTF8));
        label_11->setText(QApplication::translate("plugin", "Gripper", 0, QApplication::UnicodeUTF8));
        checkBoxPertubation->setText(QApplication::translate("plugin", "Add Pertubation", 0, QApplication::UnicodeUTF8));
        label_13->setText(QApplication::translate("plugin", "Pertubation pr. target", 0, QApplication::UnicodeUTF8));
        labelSigmaPos->setText(QApplication::translate("plugin", "Sigma_Pos:", 0, QApplication::UnicodeUTF8));
        lineEditSigmaPos->setText(QApplication::translate("plugin", "0.0003", 0, QApplication::UnicodeUTF8));
        labelSigmaR->setText(QApplication::translate("plugin", "Sigma_Rot:", 0, QApplication::UnicodeUTF8));
        lineEditsigmaRot->setText(QApplication::translate("plugin", "8", 0, QApplication::UnicodeUTF8));
        btnLoadGraspScene->setText(QApplication::translate("plugin", "1) Load Scene", 0, QApplication::UnicodeUTF8));
        btnRotateObject->setText(QApplication::translate("plugin", "2) Rotate", 0, QApplication::UnicodeUTF8));
        btnGenerateGraspTable->setText(QApplication::translate("plugin", "3) Create Grasp table", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_4), QApplication::translate("plugin", "Grasp tabel generation", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("plugin", "Device", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("plugin", "Robot", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("plugin", "Gripper", 0, QApplication::UnicodeUTF8));
        checkBoxSimulate->setText(QApplication::translate("plugin", "Simulate", 0, QApplication::UnicodeUTF8));
        btnGrasp->setText(QApplication::translate("plugin", "Grasp Object", 0, QApplication::UnicodeUTF8));
        btnTest->setText(QApplication::translate("plugin", "Test", 0, QApplication::UnicodeUTF8));
        checkBoxLogGrasp->setText(QApplication::translate("plugin", "Log Grasp Result ", 0, QApplication::UnicodeUTF8));
        checkBoxLiveUpdate->setText(QApplication::translate("plugin", "Live update of Robot pose", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("plugin", "Grasp Model", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class plugin: public Ui_plugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PLUGIN_H
