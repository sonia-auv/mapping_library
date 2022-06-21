//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// proc_mapping.cpp
//
// Code generation for function 'proc_mapping'
//

// Include files
#include "proc_mapping.h"
#include "Kdtree.h"
#include "PointCloudBundler.h"
#include "Preprocessing.h"
#include "Publisher.h"
#include "Rate.h"
#include "Subscriber.h"
#include "any1.h"
#include "nullAssignment.h"
#include "pcdenoise.h"
#include "pcdownsample.h"
#include "pcfitplane.h"
#include "pcmerge.h"
#include "planeModel.h"
#include "pointCloud.h"
#include "pointCloudArray.h"
#include "proc_mapping_data.h"
#include "proc_mapping_initialize.h"
#include "proc_mapping_types.h"
#include "quatUtilities.h"
#include "rosReadField.h"
#include "rosReadXYZ.h"
#include "rt_nonfinite.h"
#include "sensor_msgs_PointCloud2Struct.h"
#include "sensor_msgs_PointFieldStruct.h"
#include "tic.h"
#include "toc.h"
#include "coder_array.h"
#include "coder_posix_time.h"
#include "mlroscpp_pub.h"
#include "mlroscpp_rate.h"
#include <cmath>
#include <cstddef>
#include <cstring>
#include <stdio.h>
#include <string.h>

// Function Definitions
void proc_mapping()
{
  PointCloudBundler ptBundler;
  Preprocessing *b_this;
  coder::b_pointCloud b_lobj_1;
  coder::planeModel model1;
  coder::planeModel model2;
  coder::planeModel *b_model1;
  coder::planeModel *b_model2;
  coder::pointCloud lobj_3[5];
  coder::pointCloud filt;
  coder::pointCloud plane1;
  coder::pointCloud plane2;
  coder::pointCloud rawPT;
  coder::pointCloud remainCloud;
  coder::pointCloud *iobj_1;
  coder::pointCloud *output;
  coder::ros::Publisher lobj_1;
  coder::ros::Publisher *pub;
  coder::ros::Rate r;
  coder::ros::Subscriber *d_sub;
  coder::ros::b_Subscriber *e_sub;
  coder::ros::c_Subscriber *sub;
  coder::ros::d_Subscriber *b_sub;
  coder::ros::e_Subscriber *c_sub;
  coder::vision::internal::codegen::Kdtree b_lobj_2[5];
  coder::vision::internal::codegen::Kdtree lobj_5[3];
  coder::vision::internal::codegen::Kdtree lobj_2;
  coder::vision::internal::codegen::Kdtree lobj_4;
  coder::vision::internal::codegen::Kdtree *iobj_0;
  coder::array<double, 2U> b_ptBundler;
  coder::array<double, 2U> nv_tmp;
  coder::array<double, 2U> xyzi;
  coder::array<double, 1U> b_x;
  coder::array<double, 1U> outlierIndices;
  coder::array<double, 1U> r2;
  coder::array<double, 1U> x;
  coder::array<float, 2U> XYZ;
  coder::array<float, 2U> r1;
  coder::array<float, 1U> RGB;
  coder::array<bool, 1U> b_xyzi;
  coder::array<bool, 1U> r3;
  sensor_msgs_PointCloud2Struct_T pack;
  double poseMsg_Orientation_Z;
  double poseMsg_Position_X;
  double poseMsg_Position_Y;
  double poseMsg_Position_Z;
  double t10;
  double t11;
  double t12;
  double t13;
  double t17;
  double t18;
  double t19;
  double t20;
  double t21;
  double t5;
  double t6;
  double t7;
  double t8;
  double t9;
  int i;
  if (!isInitialized_proc_mapping) {
    proc_mapping_initialize();
  }
  //  Variables
  r.init();
  //  Proc_mapping startup
  //         %% ROS Node constructor
  pub = lobj_1.init();
  for (i = 0; i < 5; i++) {
    lobj_3[i].matlabCodegenIsDeleted = true;
  }
  filt.matlabCodegenIsDeleted = true;
  //         %% ROS Spin
  MATLABRate_reset(r.RateHelper);
  coder::tic(&t10, &t5);
  printf("INFO : proc mapping : Node is started. \n");
  fflush(stdout);
  printf("INFO : proc mapping : Wait for point cloud. \n");
  fflush(stdout);
  //  Instances
  //         %% PointCloudBundler Constructor
  ptBundler.bundle.set_size(3, 4);
  for (i = 0; i < 12; i++) {
    ptBundler.bundle[i] = 0.0;
  }
  ptBundler.bundle.set_size(1, 4);
  ptBundler.bundle[0] = 0.0;
  ptBundler.bundle[1] = 0.0;
  ptBundler.bundle[2] = 0.0;
  ptBundler.bundle[3] = 0.0;
  //  Subscribers
  sub = ptBundler._pobj2.init();
  ptBundler.startStopSub = sub;
  b_sub = ptBundler._pobj1.init();
  ptBundler.poseSub = b_sub;
  c_sub = ptBundler._pobj0.init();
  ptBundler.sonarSub = c_sub;
  ptBundler.lastBundleState = false;
  b_this = &ptBundler.mPreprocessing;
  //         %% Preprocessing Constructor
  //  General preprocessing value
  //              this.minIntensityState = true;
  //              this.maxIntensityState = false;
  //              this.minRangeState = true;
  //              this.maxRangeState = false;
  //              this.minIntensity = 0.07;
  //              this.maxIntensity = 1;
  //              this.minRange = 0.1;
  //              this.maxRange = 100;
  //  Subscribers
  d_sub = b_this->_pobj3.init();
  ptBundler.mPreprocessing.minIntensitySub = d_sub;
  d_sub = b_this->_pobj2.b_init();
  ptBundler.mPreprocessing.maxIntensitySub = d_sub;
  e_sub = b_this->_pobj1.init();
  ptBundler.mPreprocessing.minRangeSub = e_sub;
  e_sub = b_this->_pobj0.b_init();
  ptBundler.mPreprocessing.maxRangeSub = e_sub;
  while (1) {
    //         %% Step function
    if (ptBundler.lastBundleState && (!bundleStarted)) {
      int nx;
      unsigned int q0;
      //  Initial variables
      //  GET
      //  Record finished.
      ptBundler.lastBundleState = false;
      printf("INFO : proc mapping : Not bundling. \n");
      fflush(stdout);
      filt.matlabCodegenDestructor();
      //         %% Getters / Setters
      xyzi.set_size(ptBundler.bundle.size(0), 4);
      nx = ptBundler.bundle.size(0) * 4;
      for (i = 0; i < nx; i++) {
        xyzi[i] = ptBundler.bundle[i];
      }
      b_lobj_1.matlabCodegenIsDeleted = true;
      //  Create the point cloud and apply denoise filter plus a downsample.
      nx = xyzi.size(0);
      rawPT.Location.set_size(xyzi.size(0), 3);
      for (i = 0; i < 3; i++) {
        for (int k{0}; k < nx; k++) {
          rawPT.Location[k + rawPT.Location.size(0) * i] =
              xyzi[k + xyzi.size(0) * i];
        }
      }
      rawPT.Color.set_size(0, 0);
      rawPT.Normal.set_size(0, 0);
      nx = xyzi.size(0);
      rawPT.Intensity.set_size(xyzi.size(0));
      for (i = 0; i < nx; i++) {
        rawPT.Intensity[i] = xyzi[i + xyzi.size(0) * 3];
      }
      coder::pointclouds::internal::codegen::pc::pointCloudArray b_r;
      rawPT.RangeData.set_size(0, 0);
      rawPT.PointCloudArrayData.set_size(1, 1);
      rawPT.PointCloudArrayData[0] = b_r;
      rawPT.XLimitsInternal.set_size(0, 0);
      rawPT.YLimitsInternal.set_size(0, 0);
      rawPT.ZLimitsInternal.set_size(0, 0);
      rawPT.Kdtree = &lobj_2;
      rawPT.matlabCodegenIsDeleted = false;
      coder::pcdenoise(coder::pcdownsample(&rawPT, &b_lobj_1), &lobj_4, &filt);
      b_lobj_1.matlabCodegenDestructor();
      rawPT.matlabCodegenDestructor();
      for (i = 0; i < 5; i++) {
        lobj_3[i].matlabCodegenDestructor();
      }
      double q1[4];
      double in4[3];
      iobj_0 = &b_lobj_2[0];
      iobj_1 = &lobj_3[0];
      plane2.matlabCodegenIsDeleted = true;
      remainCloud.matlabCodegenIsDeleted = true;
      plane1.matlabCodegenIsDeleted = true;
      //  Get first wall
      coder::pcfitplane(&filt, &model1, &b_model1, x, outlierIndices);
      filt.b_select(x, &lobj_5[0], &plane1);
      filt.b_select(outlierIndices, &lobj_5[1], &remainCloud);
      coder::pcfitplane(&remainCloud, &model2, &b_model2, x, outlierIndices);
      remainCloud.b_select(x, &lobj_5[2], &plane2);
      //  Extraire les point orienté
      quatUtilities::getOrientedPointOnPlanarFace(&model1, &plane1, in4, q1);
      quatUtilities::getOrientedPointOnPlanarFace(&model2, &plane2, in4, q1);
      output = coder::pcmerge(&plane1, &plane2, &iobj_0[0], &iobj_1[0]);
      plane1.matlabCodegenDestructor();
      remainCloud.matlabCodegenDestructor();
      plane2.matlabCodegenDestructor();
      XYZ.set_size(output->Location.size(0), 3);
      nx = output->Location.size(0) * 3;
      for (i = 0; i < nx; i++) {
        XYZ[i] = static_cast<float>(output->Location[i]);
      }
      RGB.set_size(output->Intensity.size(0));
      nx = output->Intensity.size(0);
      for (i = 0; i < nx; i++) {
        RGB[i] = static_cast<float>(output->Intensity[i]);
      }
      sensor_msgs_PointCloud2Struct(&pack);
      pack.IsBigendian = false;
      pack.IsDense = true;
      pack.Header.FrameId.set_size(1, 4);
      pack.Header.FrameId[0] = 'B';
      pack.Header.FrameId[1] = 'O';
      pack.Header.FrameId[2] = 'D';
      pack.Header.FrameId[3] = 'Y';
      //  Calculate number of points
      //  Assign metadata
      pack.Height = 1U;
      pack.Width = static_cast<unsigned int>(XYZ.size(0));
      pack.PointStep = 16U;
      t10 = 16.0 * static_cast<double>(XYZ.size(0));
      if (t10 < 4.294967296E+9) {
        q0 = static_cast<unsigned int>(t10);
      } else {
        q0 = MAX_uint32_T;
      }
      pack.RowStep = q0;
      //  Assign point field data
      pack.Data.set_size(static_cast<int>(q0));
      nx = static_cast<int>(q0);
      for (i = 0; i < nx; i++) {
        pack.Data[i] = 0U;
      }
      sensor_msgs_PointFieldStruct(&(pack.Fields.data())[0]);
      pack.Fields[0].Name[0] = 'x';
      pack.Fields[0].Datatype = 7U;
      pack.Fields[0].Count = 1U;
      sensor_msgs_PointFieldStruct(&pack.Fields[1]);
      pack.Fields[1].Name[0] = 'y';
      pack.Fields[1].Datatype = 7U;
      pack.Fields[1].Count = 1U;
      sensor_msgs_PointFieldStruct(&pack.Fields[2]);
      pack.Fields[2].Name[0] = 'z';
      pack.Fields[2].Datatype = 7U;
      pack.Fields[2].Count = 1U;
      sensor_msgs_PointFieldStruct(&pack.Fields[3]);
      for (i = 0; i < 9; i++) {
        pack.Fields[3].Name[i] = cv[i];
      }
      pack.Fields[3].Datatype = 7U;
      pack.Fields[3].Count = 1U;
      pack.Fields[0].Offset = 0U;
      pack.Fields[1].Offset = 4U;
      pack.Fields[2].Offset = 8U;
      pack.Fields[3].Offset = 12U;
      //  Assign raw point cloud data in uint8 format
      i = XYZ.size(0);
      for (nx = 0; nx < i; nx++) {
        float c_x[3];
        unsigned int qY;
        unsigned char y[12];
        unsigned char b_y[4];
        t10 = ((static_cast<double>(nx) + 1.0) - 1.0) * 16.0;
        if (t10 < 4.294967296E+9) {
          q0 = static_cast<unsigned int>(t10);
        } else {
          q0 = MAX_uint32_T;
        }
        qY = q0 + 1U;
        if (q0 + 1U < q0) {
          qY = MAX_uint32_T;
        }
        c_x[0] = XYZ[nx];
        c_x[1] = XYZ[nx + XYZ.size(0)];
        c_x[2] = XYZ[nx + XYZ.size(0) * 2];
        std::memcpy((void *)&y[0], (void *)&c_x[0],
                    (unsigned int)((size_t)12 * sizeof(unsigned char)));
        for (int k{0}; k < 12; k++) {
          q0 = qY + k;
          if (q0 < qY) {
            q0 = MAX_uint32_T;
          }
          pack.Data[static_cast<int>(q0) - 1] = y[k];
        }
        std::memcpy((void *)&b_y[0], (void *)&RGB[nx],
                    (unsigned int)((size_t)4 * sizeof(unsigned char)));
        q0 = qY + 12U;
        if (qY + 12U < qY) {
          q0 = MAX_uint32_T;
        }
        pack.Data[static_cast<int>(q0) - 1] = b_y[0];
        q0 = qY + 13U;
        if (qY + 13U < qY) {
          q0 = MAX_uint32_T;
        }
        pack.Data[static_cast<int>(q0) - 1] = b_y[1];
        q0 = qY + 14U;
        if (qY + 14U < qY) {
          q0 = MAX_uint32_T;
        }
        pack.Data[static_cast<int>(q0) - 1] = b_y[2];
        q0 = qY + 15U;
        if (qY + 15U < qY) {
          q0 = MAX_uint32_T;
        }
        pack.Data[static_cast<int>(q0) - 1] = b_y[3];
        //  NOTE: The 16th byte remains empty
      }
      MATLABPUBLISHER_publish(pub->PublisherHelper, &pack);
    } else {
      bool out;
      //  Recording or waiting.
      //  Initial variables
      //  GET
      out = newSonarMsg;
      if (out && bundleStarted) {
        int nx;
        //  Initial variables
        //  GET
        ptBundler.sonarSub->get_LatestMessage(&pack);
        ptBundler.poseSub->get_LatestMessage(
            &poseMsg_Position_X, &poseMsg_Position_Y, &poseMsg_Position_Z, &t11,
            &t12, &poseMsg_Orientation_Z, &t13);
        //         %% Adding to the point cloud.
        printf("INFO : proc mapping : Append to point cloud. \n");
        fflush(stdout);
        //  scan = rosReadLidarScan(sonarMsg);
        //  Getting the sub pose.
        printf("INFO : proc mapping : Pose received. \n");
        fflush(stdout);
        xyzi.set_size(static_cast<int>(pack.Width), 4);
        nx = static_cast<int>(pack.Width) << 2;
        for (i = 0; i < nx; i++) {
          xyzi[i] = 0.0;
        }
        coder::rosReadXYZ(pack.Height, pack.Width, pack.Fields, pack.PointStep,
                          pack.Data, XYZ);
        nx = XYZ.size(0);
        for (i = 0; i < 3; i++) {
          for (int k{0}; k < nx; k++) {
            xyzi[k + xyzi.size(0) * i] = XYZ[k + XYZ.size(0) * i];
          }
        }
        //  Temporary swap.
        //              v = xyzi(:, 1);
        //              xyzi(:, 1) = xyzi(:, 2);
        //              xyzi(:, 2) = v;
        coder::rosReadField(pack.Height, pack.Width, pack.Fields,
                            pack.PointStep, pack.Data, r1);
        nv_tmp.set_size(r1.size(0), r1.size(1));
        nx = r1.size(0) * r1.size(1);
        for (i = 0; i < nx; i++) {
          nv_tmp[i] = r1[i];
        }
        nx = xyzi.size(0);
        for (i = 0; i < nx; i++) {
          xyzi[i + xyzi.size(0) * 3] = nv_tmp[i];
        }
        // minIntensityState maxIntensityState minRangeState maxRangeState;
        //  Initial variables
        //  GET
        //                          out2 = minIntensityState;
        // minIntensityState maxIntensityState minRangeState maxRangeState;
        //  Initial variables
        //  GET
        //                          out2 = maxIntensityState;
        // minIntensityState maxIntensityState minRangeState maxRangeState;
        //  Initial variables
        //  GET
        //                          out2 = minRangeState;
        // minIntensityState maxIntensityState minRangeState maxRangeState;
        //  Initial variables
        //  GET
        //                          out2 = maxRangeState;
        nx = xyzi.size(0);
        x.set_size(xyzi.size(0));
        for (i = 0; i < nx; i++) {
          t10 = xyzi[i];
          x[i] = t10 * t10;
        }
        nx = xyzi.size(0);
        outlierIndices.set_size(xyzi.size(0));
        for (i = 0; i < nx; i++) {
          t10 = xyzi[i + xyzi.size(0)];
          outlierIndices[i] = t10 * t10;
        }
        nx = xyzi.size(0);
        r2.set_size(xyzi.size(0));
        for (i = 0; i < nx; i++) {
          t10 = xyzi[i + xyzi.size(0) * 2];
          r2[i] = t10 * t10;
        }
        if (x.size(0) == 1) {
          i = outlierIndices.size(0);
        } else {
          i = x.size(0);
        }
        if ((x.size(0) == outlierIndices.size(0)) && (i == r2.size(0))) {
          nx = x.size(0);
          for (i = 0; i < nx; i++) {
            x[i] = (x[i] + outlierIndices[i]) + r2[i];
          }
        } else {
          b_binary_expand_op(x, outlierIndices, r2);
        }
        nx = x.size(0);
        for (int k{0}; k < nx; k++) {
          x[k] = std::sqrt(x[k]);
        }
        nx = xyzi.size(0);
        b_x.set_size(xyzi.size(0));
        for (i = 0; i < nx; i++) {
          t10 = xyzi[i];
          b_x[i] = t10 * t10;
        }
        nx = xyzi.size(0);
        outlierIndices.set_size(xyzi.size(0));
        for (i = 0; i < nx; i++) {
          t10 = xyzi[i + xyzi.size(0)];
          outlierIndices[i] = t10 * t10;
        }
        nx = xyzi.size(0);
        r2.set_size(xyzi.size(0));
        for (i = 0; i < nx; i++) {
          t10 = xyzi[i + xyzi.size(0) * 2];
          r2[i] = t10 * t10;
        }
        if (b_x.size(0) == 1) {
          i = outlierIndices.size(0);
        } else {
          i = b_x.size(0);
        }
        if ((b_x.size(0) == outlierIndices.size(0)) && (i == r2.size(0))) {
          nx = b_x.size(0);
          for (i = 0; i < nx; i++) {
            b_x[i] = (b_x[i] + outlierIndices[i]) + r2[i];
          }
        } else {
          b_binary_expand_op(b_x, outlierIndices, r2);
        }
        nx = b_x.size(0);
        for (int k{0}; k < nx; k++) {
          b_x[k] = std::sqrt(b_x[k]);
        }
        if (xyzi.size(0) == 1) {
          i = x.size(0);
        } else {
          i = xyzi.size(0);
        }
        if ((xyzi.size(0) == x.size(0)) && (i == b_x.size(0))) {
          nx = xyzi.size(0);
          b_xyzi.set_size(xyzi.size(0));
          for (i = 0; i < nx; i++) {
            t10 = xyzi[i + xyzi.size(0) * 3];
            b_xyzi[i] =
                ((t10 < minIntensityValue) || (t10 > maxIntensityValue) ||
                 (x[i] < minRangeValue) || (b_x[i] > maxRangeValue));
          }
          coder::any(b_xyzi, r3);
        } else {
          binary_expand_op(r3, xyzi, minIntensityValue, maxIntensityValue, x,
                           minRangeValue, b_x, maxRangeValue);
        }
        coder::internal::nullAssignment(xyzi, r3);
        //              rowsToDelete = any(xyzi(:,4) < 0.07 |
        //              sqrt(xyzi(:,1).^2+xyzi(:,2).^2+xyzi(:,3).^2) > 0.1, 2);
        //              xyzi(rowsToDelete, :) = [];
        i = xyzi.size(0);
        if (xyzi.size(0) - 1 >= 0) {
          t5 = t11 * t12 * 2.0;
          t6 = t11 * poseMsg_Orientation_Z * 2.0;
          t7 = t12 * poseMsg_Orientation_Z * 2.0;
          t8 = t13 * t11 * 2.0;
          t9 = t13 * t12 * 2.0;
          t10 = t13 * poseMsg_Orientation_Z * 2.0;
          t11 = t11 * t11 * 2.0;
          t12 = t12 * t12 * 2.0;
          t13 = poseMsg_Orientation_Z * poseMsg_Orientation_Z * 2.0;
          t17 = t5 + t10;
          t18 = t6 + t9;
          t19 = t7 + t8;
          t20 = t5 + -t10;
          t21 = t6 + -t9;
          t7 += -t8;
          t8 = (t11 + t12) - 1.0;
          t9 = (t11 + t13) - 1.0;
          t6 = (t12 + t13) - 1.0;
        }
        for (nx = 0; nx < i; nx++) {
          double in4[3];
          in4[0] = xyzi[nx];
          in4[1] = xyzi[nx + xyzi.size(0)];
          // sonar2NED
          //     OUT1 = sonar2NED(IN1,IN2,IN3,IN4)
          //     This function was generated by the Symbolic Math Toolbox
          //     version 9.1. 11-May-2022 11:36:02
          xyzi[nx] = (((((poseMsg_Position_X - 0.358 * t6) + 0.0 * t20) +
                        -0.118 * t18) -
                       t6 * in4[0]) +
                      t20 * in4[1]) +
                     t18 * 0.0;
          xyzi[nx + xyzi.size(0)] =
              (((((poseMsg_Position_Y + 0.358 * t17) - 0.0 * t9) +
                 -0.118 * t7) +
                t17 * in4[0]) -
               t9 * in4[1]) +
              t7 * 0.0;
          xyzi[nx + xyzi.size(0) * 2] =
              (((((poseMsg_Position_Z + 0.358 * t21) + 0.0 * t19) -
                 -0.118 * t8) +
                t21 * in4[0]) +
               t19 * in4[1]) -
              t8 * 0.0;
        }
        b_ptBundler.set_size(ptBundler.bundle.size(0) + xyzi.size(0), 4);
        for (i = 0; i < 4; i++) {
          nx = ptBundler.bundle.size(0);
          for (int k{0}; k < nx; k++) {
            b_ptBundler[k + b_ptBundler.size(0) * i] =
                ptBundler.bundle[k + ptBundler.bundle.size(0) * i];
          }
        }
        nx = xyzi.size(0);
        for (i = 0; i < 4; i++) {
          for (int k{0}; k < nx; k++) {
            b_ptBundler[(k + ptBundler.bundle.size(0)) +
                        b_ptBundler.size(0) * i] = xyzi[k + xyzi.size(0) * i];
          }
        }
        ptBundler.bundle.set_size(b_ptBundler.size(0), 4);
        nx = b_ptBundler.size(0) * 4;
        for (i = 0; i < nx; i++) {
          ptBundler.bundle[i] = b_ptBundler[i];
        }
        //  Initial variables
        //  SET
        newSonarMsg = false;
      }
      //  Initial variables
      //  GET
      ptBundler.lastBundleState = bundleStarted;
      //  fprintf('INFO : proc mapping : Bundling or waiting. \n');
    }
    MATLABRate_sleep(r.RateHelper);
    coder::toc(r.PreviousPeriod.tv_sec, r.PreviousPeriod.tv_nsec);
    coder::tic(&r.PreviousPeriod.tv_sec, &r.PreviousPeriod.tv_nsec);
  }
}

// End of code generation (proc_mapping.cpp)
