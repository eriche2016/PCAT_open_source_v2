#ifndef RVIZ_CLOUD_ANNOTATION_PLUGIN_H
#define RVIZ_CLOUD_ANNOTATION_PLUGIN_H

#include <rviz_cloud_annotation/UndoRedoState.h>
// by hxw 
#include "rviz_cloud_annotation.h" // already include "rviz_cloud_annotation_params.h"
// by hxw
#include "rviz_cloud_annotation_params.h"

// QT
#include <QAction>
#include <QBoxLayout>
#include <QButtonGroup>
#include <QGridLayout>
#include <QKeySequence>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPushButton>
#include <QShortcut>
#include <QSlider>
#include <QStyle>
#include <QToolButton>

// STL
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

// ROS
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>

// PCL
#include <pcl/common/colors.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt32.h>

#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt64MultiArray.h>

#include <stdint.h>
#include <QFileDialog>
#include <vector>

// OpenCv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"

#include <rviz_cloud_annotation/UndoRedoState.h>

// by hxw 
#include <dirent.h>

class QLabel;
class QPushButton;
class QButtonGroup;
class QLineEdit;
class QAction;
class QToolButton;
class QSlider;
class QMenu;

namespace pcl
{
class RGB;
}

namespace rviz_cloud_annotation
{
// by hxw 
typedef std::vector<std::string> StringVector;

class QRVizCloudAnnotation : public rviz::Panel
{
  Q_OBJECT;

  typedef uint64_t uint64;
  typedef int64_t int64;
  typedef uint32_t uint32;
  typedef int32_t int32;
  typedef std::vector<uint64> Uint64Vector;
  typedef std::vector<float> FloatVector;
  typedef std::vector<int64_t> Int64Vector;
  typedef std::vector<bool> BoolVector;
  typedef std::vector<QPushButton *> PQPushButtonVector;
  
public:
  QRVizCloudAnnotation(QWidget *parent = NULL);
  virtual ~QRVizCloudAnnotation();

private Q_SLOTS:
  void onSetEditMode(int mode);
  void onSetToolType(int type);

  void onSetAnnotationType(uint32 annotation_type);

  void onLabelButtonSelected(int id);
  void onPlusLabel();
  void onMinusLabel();
  void onPageUp();
  void onPageDown();

  void onAutoPlane();

  void onSave();
  void onRestore();
  void onClear();
  void onNew();
  void onClearCurrent();

  void onUndo();
  void onRedo();

  void onSendName();

  void onViewCloudToggled(const bool checked);
  void onViewControlPointsToggled(const bool checked);
  void onViewLabelsToggled(const bool checked);

  void onGotoFirstUnused();
  void onGotoLastUnused();
  void onGotoFirst();
  void onGotoNextUnused();

  void onSmallerPoints();
  void onBiggerPoints();
  void onResetPointsSize();

  void onControlYawSliderMoved(int new_value);
  void onControlYawSliderSet(int new_value);
  void onSetControlMaxYaw(const std_msgs::Int32 &msg);
  void onSetControlMinYaw(const std_msgs::Int32 &msg);
  void onYawInc();
  void onYawDec();

  void onBiasX1();
  void onBiasX2();
  void onBiasX3();
  void onBiasX4();
  void onBiasY1();
  void onBiasY2();
  void onBiasY3();
  void onBiasY4();
  void onBiasZ1();
  void onBiasZ2();
  void onBiasZ3();
  void onBiasZ4();

  void onSetBiasZero(const std_msgs::Empty &msg);

  void onSetObjectId(const std_msgs::Int32 &msg);
  void onChangeObjectId();

  void onControlPointWeightSliderMoved(int new_value);
  void onControlPointWeightSliderSet(int new_value);
  void onControlPointWeightInc();
  void onControlPointWeightDec();
  void onControlPointWeightMax();
  void onControlPointWeightMin();
  void onSetControlPointMaxWeight(const std_msgs::Int32 &msg);

private:
  void SetCurrentEditMode(const uint64 mode);
  // define a functions to read data v1 
  StringVector read_image_label_v1(std::string imageDir) {
      StringVector label_files;
      struct dirent *entry; 
      DIR * dir;
      if (dir = opendir(imageDir.c_str())) {
        while (entry = readdir(dir)) {
          std::string name = entry->d_name; 
          if (name != "." & name != "..") {
            label_files.push_back(imageDir + name.c_str());
            // std::cout << "hello" << imageDir + name.c_str() << std::endl;
          }
        }
        closedir(dir);
      } 
      /*
      StringVector label_files;

      for (int k = 0; k < 2 ; k++) {
        std::string name = imageDir + std::string("1") + std::string(".jpg");
        label_files.push_back(name.c_str());
      }
      */ 
      return label_files; 
  } 
 

  StringVector read_image_label_v2(std::string imageDir) {
    ROS_INFO("reading image path in a folder and store in a string vector.");
    StringVector label_files;
    StringVector img_label_files;
    std::ifstream ifile;
    std::string m_log_file = "/home/ps/lidar_annotation/annotation.log"; // hxw 
    ifile.open(m_log_file.c_str());

    std::string s;
    while (std::getline(ifile, s)) {
      label_files.push_back(s);
      // ROS_INFO("rviz_cloud_annotation: label_files: %s", s.c_str());
    }

    PointCloudFilesTool *pcft;
    StringVector m_pcd_files;
    pcft->getFilesList("/home/ps/lidar_annotation/pcd/", m_pcd_files);
    for (int k = 0; k < m_pcd_files.size(); k++) {
      bool ifNew = true;
      std::string FILE_NAME = m_pcd_files[k];

      std::string name = "/home/ps/lidar_annotation/pcd/" + FILE_NAME;
      ROS_INFO("current point cloud: %s", name.c_str());

      for (int i = 0; i < label_files.size(); i++) {
        ROS_INFO("current label files: %s", label_files[i].c_str());
        if (name.compare(label_files[i]) == 0) {
          ifNew = false;
          break;
        }
      }
      if (ifNew) {
        std::string index = FILE_NAME.substr(0, FILE_NAME.length() - 4);

        name = "/home/ps/lidar_annotation/image/" + index + std::string(".jpg");
        // by hxw
        ROS_INFO("current image For current point cloud: %s", name.c_str());

        img_label_files.push_back(name.c_str());
      }
    }
    return img_label_files;
  }

  void FillColorPageButtons();
  void FillPointCounts(uint type);
  void FillColorPageButtonStylesheet();

  void SetCurrentLabel(const uint64 label, const uint64 page);

  void onSetCurrentLabel(const std_msgs::UInt32 &label);
  void onSetEditMode2(const std_msgs::UInt32 &mode);
  void onPointCountUpdate(const std_msgs::UInt64MultiArray &counters);
  void onUndoRedoState(const rviz_cloud_annotation::UndoRedoState &msg);

  void onSetName(const std_msgs::String &name);

  uint64 GetPageForLabel(const uint64 label) const;
  uint64 GetLabelFromPageAndId(const uint64 page, const int id) const;
  uint64 GetFirstLabelForPage(const uint64 page) const;
  uint64 GetLastLabelForPage(const uint64 page) const;

  void SetUndoText(const std::string &text);
  void SetRedoText(const std::string &text);

  void onNextObject();

  void onPreObject();

  void showImageLabel(std::string imagePath);

  void FirstUsedTime(time_t &begin)
  {
    const char *timefile_name = "time.txt";
    std::ifstream infile;
    std::ofstream outfile;
    infile.open(timefile_name);
    outfile.open(timefile_name, std::ios::app);
    std::string timeS;
    if (infile)
    {
      std::getline(infile, timeS);
      begin = atoi(timeS.c_str());
    }
    else
    {
      outfile << begin << std::endl;
      outfile.close();
    }
  }

  static void ColorToHex(const pcl::RGB &color, char hex[7]);

  uint64 m_current_edit_mode;

  int32 bbox_id = 0;
  int32 lane_id = 0;
  int32 kerb_id = 0;

  // 0 for the eraser
  uint64 m_current_label;
  uint64 m_current_page;

  ros::NodeHandle m_nh;
  ros::Publisher m_set_edit_mode_pub;
  ros::Publisher m_set_current_label_pub;

  ros::Publisher m_save_pub;
  ros::Publisher m_restore_pub;
  ros::Publisher m_clear_pub;
  ros::Publisher m_new_pub;

  ros::Publisher m_goto_first_unused_pub;
  ros::Publisher m_goto_last_unused_pub;
  ros::Publisher m_goto_first_pub;
  ros::Publisher m_goto_next_unused_pub;

  ros::Subscriber m_set_edit_mode_sub;
  ros::Subscriber m_set_current_label_sub;

  ros::Publisher m_set_name_pub;
  ros::Subscriber m_set_name_sub;

  ros::Publisher m_view_cloud_pub;
  ros::Publisher m_view_labels_pub;
  ros::Publisher m_view_control_points_pub;

  ros::Publisher m_next_pub;
  ros::Publisher m_pre_pub;

  ros::Publisher m_redo_pub;
  ros::Publisher m_undo_pub;
  ros::Subscriber m_undo_redo_state_sub;

  ros::Publisher m_point_size_change_pub;

  ros::Publisher m_control_points_weight_pub;
  ros::Subscriber m_control_point_max_weight_sub;

  ros::Publisher m_bias_pub;
  ros::Subscriber m_bias_zero_sub;

  ros::Publisher m_yaw_pub;
  ros::Subscriber m_yaw_max_sub;
  ros::Subscriber m_yaw_min_sub;

  ros::Publisher m_bbox_occluded_pub;

  ros::Publisher m_tool_type_pub;

  ros::Publisher m_annotation_type_pub;
  ros::Subscriber m_object_id_sub;

  ros::Publisher m_auto_plane_pub;

  image_transport::Publisher m_image_label_pub;

  QPushButton *m_edit_none_button;
  QPushButton *m_edit_control_point_button;
  QPushButton *m_edit_eraser_button;
  QPushButton *m_edit_color_picker_button;
  QButtonGroup *m_toolbar_group;

  QPushButton *m_kerb_button;
  QPushButton *m_lane_button;
  QPushButton *m_plane_button;
  QPushButton *m_bbox_button;
  QButtonGroup *m_annotation_type_group;

  QPushButton *m_tool_single_button;
  QPushButton *m_tool_shallow_square_button;
  QPushButton *m_tool_deep_square_button;
  QPushButton *m_tool_shallow_poly_button;
  QButtonGroup *m_tooltype_group;

  QAction *m_prev_page_action;
  QAction *m_next_page_action;
  QAction *m_next_label_action;
  QAction *m_prev_label_action;

  QAction *m_prev_weight_action;
  QAction *m_next_weight_action;
  QAction *m_min_weight_action;
  QAction *m_max_weight_action;
  QMenu *m_weight_menu;

  QAction *m_auto_plane_action;

  QAction *m_X1_bias_action;
  QAction *m_X2_bias_action;
  QAction *m_X3_bias_action;
  QAction *m_X4_bias_action;
  QAction *m_Y1_bias_action;
  QAction *m_Y2_bias_action;
  QAction *m_Y3_bias_action;
  QAction *m_Y4_bias_action;
  QAction *m_Z1_bias_action;
  QAction *m_Z2_bias_action;
  QAction *m_Z3_bias_action;
  QAction *m_Z4_bias_action;
  QMenu *m_bias_menu;

  QAction *m_prev_yaw_action;
  QAction *m_next_yaw_action;
  QAction *m_max_yaw_action;
  QAction *m_min_yaw_action;
  QMenu *m_yaw_menu;

  QAction *m_undo_action;
  QAction *m_redo_action;

  QLabel *m_current_page_label;

  QLabel *m_object_id;

  QLabel *m_current_control_point_weight_label;
  QSlider *m_current_control_point_weight_slider;
  uint32 m_control_point_weight_max;

  QLabel *m_current_yaw_label;
  QSlider *m_current_yaw_slider;
  int32 m_yaw_max;
  int32 m_yaw_min;

  PQPushButtonVector m_page_buttons;
  QButtonGroup *m_page_button_group;

  Uint64Vector m_point_counters;
  ros::Subscriber m_point_count_update_sub;

  QLineEdit *m_set_name_edit;

  uint64 m_color_cols_per_page;
  uint64 m_color_rows_per_page;

  FloatVector m_bias;
  const float bias_step = 0.05;

  const uint BBOX = 0u;
  const uint PLANE = 1u;
  const uint KERB = 2u;
  const uint LANE = 3u;
  int fileid = 0; 

  uint ANNOTATION_TYPE = BBOX;

  // by hxw 
  // StringVector m_image_files = read_image_label_v1("/home/ps/lidar_annotation/image/"); 
  StringVector m_image_files = read_image_label_v2("/home/ps/lidar_annotation/image/"); 
  
  // now let us write in m_image_files in a hard code manner 
};

}  // namespace rviz_cloud_annotation

#endif  // RVIZ_CLOUD_ANNOTATION_PLUGIN_H
