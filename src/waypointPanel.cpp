#include "waypointPanel.h"
#include <ui_waypointPanel.h>
#include <vector>
#include <QInputDialog>
#include <QLineEdit>
#include <cstddef>
#include <std_msgs/String.h>
#include <ros/master.h>
#include <thread>

namespace waypoints
{
    waypointPanel::waypointPanel(QWidget *parent) : rviz::Panel(parent),
                                                    ui(new Ui::waypointPanel)
    {
        ros::V_string nodes_name;
        ros::master::getNodes(nodes_name);
        bool wp_srv = false;
        for (std::string nh : nodes_name)
        {
            if (nh == "/waypoints_server")
            {
                ROS_INFO("node %s running", nh.c_str());
                wp_srv = true;
            }
        }
        if (!wp_srv)
        {
            ROS_INFO("node /waypoints_server  is  NOT running");
        }

        else
        {

            ui->setupUi(this);

            pub_rviz = n.advertise<geometry_msgs::PoseArray>("waypoint_server/waypoints_rviz", 1);
            sub = n.subscribe("waypoint_server/waypoints", 3, &waypointPanel::Callback, this);

            connect(ui->deleteWpButton, SIGNAL(clicked(bool)), this, SLOT(onDeleteWaypoint()));
            connect(ui->newGroupButton, SIGNAL(clicked(bool)), this, SLOT(onNewGroup()));
            connect(ui->deleteGroupButton, SIGNAL(clicked(bool)), this, SLOT(onDeleteGroup()));
            connect(ui->add2GroupButton, SIGNAL(clicked(bool)), this, SLOT(onAdd2Group()));
            connect(ui->delete2GroupButton, SIGNAL(clicked(bool)), this, SLOT(onDelete2Group()));
            connect(ui->stopGroupButton, SIGNAL(clicked(bool)), this, SLOT(onStopGroup()));
            connect(ui->runGroupButton, SIGNAL(clicked(bool)), this, SLOT(onRunGroup()));
            connect(ui->runWpButton, SIGNAL(clicked(bool)), this, SLOT(onRunWp()));
            connect(ui->groupBox, SIGNAL(currentTextChanged(const QString &)), this, SLOT(onGroupBox(const QString &)));
            connect(ui->loopcheckBox, SIGNAL(stateChanged(int)), this, SLOT(onLoop(int)));

            connect(ui->saveFileButton, SIGNAL(clicked(bool)), this, SLOT(onSaveFile()));
            connect(ui->loadFileButton, SIGNAL(clicked(bool)), this, SLOT(onLoadFile()));
        }
    }

    waypointPanel::~waypointPanel()
    {
        delete ui;
    }
    void waypointPanel::PublishWp()
    {
        geometry_msgs::PoseArray arrayWp_rviz;
        arrayWp_rviz.header.stamp = ros::Time::now();
        arrayWp_rviz.header.frame_id = "map";

        arrayWp_rviz.poses.resize(wp_map.size());
        int i = 0;
        ui->listWp->clear();
        for (auto it = wp_map.begin(); it != wp_map.end(); ++it)
        {
            ui->listWp->addItem(it->first.c_str());
            arrayWp_rviz.poses[i] = it->second.pose;
            i++;
        }
        ui->listGroup->clear();
        ui->groupBox->clear();
        for (auto it = groups.begin(); it != groups.end(); ++it)
        {
            ui->groupBox->addItem(it->first.c_str());
        }
        pub_rviz.publish(arrayWp_rviz);
        int index = ui->groupBox->findText(textGroupBox.c_str());
        ui->groupBox->setCurrentIndex(index);
        ROS_INFO("Update waypoint server");
    }

    void waypointPanel::Callback(waypoints::waypointArray wp_msg)
    {
        ROS_INFO("Received wp_msg");
        wp_map.clear();
        for (waypoints::waypoint_msg wp : wp_msg.waypoints)
        {
            wp_map[wp.name] = wp;
        }
        groups.clear();
        for (waypoints::waypoint_group gr : wp_msg.groups)
        {
            groups[gr.name] = gr;
        }
        PublishWp();
    }

    void waypointPanel::onDeleteWaypoint()
    {
        std::string name = ui->listWp->currentItem()->text().toStdString();
        std::string msg = "rosservice call /waypoint_server/delete_wp \"wp_name: '" + name + "'\"";
        //to not change view
        textGroupBox = ui->groupBox->currentText().toStdString();

        system(msg.c_str());
    }

    void waypointPanel::onNewGroup()
    {

        std::string name_gp = QInputDialog::getText(nullptr, "Group Name", "Name:", QLineEdit::Normal, "").toStdString();
        if (name_gp.empty())
        {
            ROS_INFO("Put a valid name");
        }
        else
        {
            textGroupBox = ui->groupBox->currentText().toStdString();
            std::string msg = "rosservice call /waypoint_server/groups_wp \"option: 'add' \ngroup_name : '" + name_gp + "'\"";
            system(msg.c_str());
        }
    }

    void waypointPanel::onDeleteGroup()
    {
        if (groups.size() > 0)
        {
            std::string name_gp = ui->groupBox->currentText().toStdString();
            std::string msg = "rosservice call /waypoint_server/groups_wp \"option: 'delete' \ngroup_name : '" + name_gp + "'\"";
            textGroupBox = ui->groupBox->currentText().toStdString();
            system(msg.c_str());
        }
    }

    void waypointPanel::onAdd2Group()
    {
        if (ui->listWp->currentRow() != -1 && groups.size() > 0)
        {
            std::string wp = ui->listWp->currentItem()->text().toStdString();
            std::string group = ui->groupBox->currentText().toStdString();
            textGroupBox = group;
            std::string msg = "rosservice call /waypoint_server/wp_2_group \"option: 'add'\ngroup_name: '" + group + "'\nwp_name: '" + wp + "'\"";
            system(msg.c_str());
        }
    }

    void waypointPanel::onDelete2Group()
    {
        if (ui->listGroup->currentRow() != -1 && groups.size() > 0)
        {
            std::string wp = ui->listGroup->currentItem()->text().toStdString();
            std::string pos = std::to_string(ui->listGroup->currentRow());
            std::string group = ui->groupBox->currentText().toStdString();
            textGroupBox = group;
            std::string msg = "rosservice call /waypoint_server/wp_2_group \"option: 'delete'\ngroup_name: '" + group + "'\nwp_name: '" + wp + "'\npos: " + pos + "\"";
            system(msg.c_str());
            ROS_INFO("remove wp %s from group %s", wp.c_str(), group.c_str());
        }
    }

    void waypointPanel::onRunGroup()
    {
        auto run = [](std::string msg) { system(msg.c_str()); };
        if (groups.size() > 0)
        {
            std::string gr_name = ui->groupBox->currentText().toStdString();
            std::string index = std::to_string(ui->listGroup->currentRow());
            bool check = ui->loopcheckBox->isChecked();
            std::string loop;
            if (check)
            {
                ui->runGroupButton->setDisabled(true);
                loop = "true";
            }

            else
                loop = "false";

            std::string msg = "rosservice call /waypoint_server/run_wp \"wp_name: '' \ngr_name: '" + gr_name + "'\n" + "loop: " + loop + "\nindex: " + index + "\"";
            std::thread t1(run, msg);
            t1.detach();

            ROS_INFO("Run Group");
        }
    }

    void waypointPanel::onRunWp()
    {

        auto run = [](std::string msg) { system(msg.c_str()); };
        if (ui->listWp->currentRow() != -1)
        {
            std::string wp = ui->listWp->currentItem()->text().toStdString();
            std::string msg = "rosservice call /waypoint_server/run_wp \"wp_name: '" + wp + "'\ngr_name: ''\nloop: false \nindex: 0 \"";
            std::thread t1(run, msg);
            t1.detach();
            ROS_INFO("Run Wp");
        }
    }

    void waypointPanel::onGroupBox(const QString &group)
    {
        ui->listGroup->clear();
        if (groups.size() > 0)
        {
            for (auto it = groups[group.toStdString()].wp_list.begin(); it != groups[group.toStdString()].wp_list.end(); ++it)
            {
                std::string item = *it;
                ui->listGroup->addItem(item.c_str());
            }
        }
    }

    void waypointPanel::onStopGroup()
    {
        std::string msg = "rosservice call /waypoint_server/stop_wp";
        ui->runGroupButton->setDisabled(false);
        system(msg.c_str());
        ROS_INFO("Stop Movement");
    }

    void waypointPanel::onSaveFile()
    {
        std::string name_file = QInputDialog::getText(nullptr, "File Name", "Name:", QLineEdit::Normal, "").toStdString();
        if (name_file.empty())
        {
            ROS_INFO("Put a valid name");
        }
        else
        {
            std::string msg = "rosservice call /waypoint_server/save_wp \"file_name: '" + name_file + "'\"";
            system(msg.c_str());
            ROS_INFO("Save File %s_wp.txt and %s_gr.txt ", name_file.c_str(), name_file.c_str());
        }
    }
    void waypointPanel::onLoadFile()
    {
        std::string name_file = QInputDialog::getText(nullptr, "File Name", "Name:", QLineEdit::Normal, "").toStdString();
        if (name_file.empty())
        {
            ROS_INFO("Put a valid name");
        }
        else
        {
            std::string msg = "rosservice call /waypoint_server/load_wp \"file_name: '" + name_file + "'\"";
            system(msg.c_str());
            ROS_INFO("Load file %s.txt", name_file.c_str());
        }
    }
    void waypointPanel::onLoop(int state)
    {
    }
} // end namespace waypoints
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(waypoints::waypointPanel, rviz::Panel)
