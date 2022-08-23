#ifndef ETRC22_ETRC_INFO_H_
#define ETRC22_ETRC_INFO_H_

#include <list>
#include <algorithm>
#include <climits>
#include <math.h>
#include <tuple>

#include "device_io.h"
#include "info_type.h"

const int kCourseParamNum = 1133; //経路追従

class Luminous {
 public:
  Luminous(SensorIo* sensor_io);
  void Update();
  Color JudgeColor(Hsv _hsv);//paku
  Color color_;
  Rgb rgb_;
  Hsv hsv_;

//////////////////////paku////////////////////////
  int max;
  int min;
  int MAX_R = 194;
  int MAX_G = 211;
  int MAX_B = 222;

  int MIN_R = 9;
  int MIN_G = 11;
  int MIN_B = 12;

  int MAX_Y_h = 30;
  int MAX_G_h = 140;
  int MAX_B_h = 245;
  int MAX_R_h = 355;
//////////////////////paku////////////////////////


 private:
  void SetColorReference(Color c, Hsv hsv);
  void UpdateRgb();
  void UpdateHsv();
  void UpdateColor();
  SensorIo* sensor_io_;
  Hsv color_ref_[kColorNum];
  char str_[256];
};

class Odometry {
  public:
   Odometry(MotorIo* motor_io);
   void Update();
   double  A = 0;
   double theta = 0;
   double x = 0;
   double y = 0;
   double correct_distance = 0;
   double direction = 0;
   double deg_theta = 0;
   double theta_wa = 0;
   double correct_deg_theta = 0;
   double sum_correct_theta = 0;

  private:
   MotorIo* motor_io_;
   const double R = 50;
   const double D = 126;
   int32_t counts_r_;
   int32_t counts_l_;
   int curr_index = 0;
   int32_t counts_rs[100000] = {};
   int32_t counts_ls[100000] = {};
   double before_x = 0;
   double before_y = 0;
   double difference_x = 0;
   double difference_y = 0;
};

class PurePursuit {
  public:
   PurePursuit(MotorIo* motor_io);
   double x, y, yaw;
   void Update(double x, double y);
   double target_distance = 0;
   double difference_rad = 0;

  private:
   MotorIo* motor_io_; 
   Odometry* odometry_;
   double calc_distance(double point_x, double point_y);
   std::tuple<int, double> pursuit_control(int ind);
   std::tuple<int, double> search_target_index();

   int ind;
   int target_ind;
   int pre_point_index= INT_MAX;
   const double lf = 65;
   double target_direction = 0;
   double direction_odo;
   double p_lf;//使ってない
   double delta = 0;

  /////////////////////////////追従させる経路////////////////////////////////////////
   float course_x[kCourseParamNum] = {0,6.01,12.02,18.03,24.04,30.05,36.06,42.07,48.08,54.09,60.1,66.11,72.12,78.13,84.14,90.15,96.16,102.17,108.18,114.19,120.2,126.21,132.22,138.23,144.24,150.25,156.26,162.27,168.28,174.29,180.3,186.31,192.32,198.33,204.34,210.35,216.36,222.37,228.38,234.39,240.4,246.41,252.42,258.43,264.44,270.45,276.46,282.47,288.48,294.49,300.5,306.51,312.52,318.53,324.54,330.55,336.56,342.57,348.58,354.59,360.6,366.61,372.62,378.63,384.64,390.65,396.66,402.67,408.68,414.69,420.7,426.71,432.72,438.73,444.74,450.75,456.76,462.77,468.78,474.79,480.8,486.81,492.82,498.83,504.84,510.85,516.86,522.87,528.88,534.89,540.9,546.91,552.92,558.93,564.94,570.95,576.96,582.97,588.98,594.99,601,607.01,613.02,619.03,625.04,631.05,637.06,643.07,649.08,655.09,661.1,667.11,673.12,679.13,685.14,691.15,697.16,703.17,709.18,715.19,721.2,727.21,733.22,739.23,745.24,751.25,757.26,763.27,769.28,775.29,781.3,787.31,793.32,799.33,805.34,811.35,817.36,823.37,829.38,835.39,841.4,847.41,853.42,859.43,865.44,871.45,877.46,883.47,889.48,895.49,901.5,907.51,913.52,919.53,925.54,931.55,937.56,943.57,949.58,955.59,961.6,967.61,973.62,979.63,985.64,991.65,997.66,1003.67,1009.68,1015.69,1021.7,1027.71,1033.72,1039.73,1045.74,1051.75,1057.76,1063.77,1069.78,1075.79,1081.8,1087.81,1093.82,1099.83,1105.84,1111.85,1117.86,1123.87,1129.88,1135.89,1141.9,1147.91,1153.92,1159.93,1165.94,1171.95,1177.96,1183.97,1189.98,1195.99,1202,1208.01,1214.02,1220.03,1226.04,1232.05,1238.06,1244.07,1250.08,1256.09,1262.1,1268.11,1274.12,1280.13,1286.14,1292.15,1298.16,1304.17,1310.18,1316.19,1322.2,1328.21,1334.22,1340.23,1346.24,1352.25,1358.26,1364.27,1370.28,1376.29,1382.3,1388.31,1394.32,1400.33,1406.34,1412.35,1418.36,1424.37,1430.38,1436.39,1442.4,1448.41,1454.42,1460.43,1466.44,1472.45,1478.46,1484.47,1490.48,1496.49,1502.5,1508.51,1514.52,1520.53,1526.54,1532.55,1538.56,1544.57,1550.58,1556.59,1562.6,1568.61,1574.62,1580.63,1586.64,1592.65,1598.66,1604.67,1604.67,1610.68,1616.69,1616.69,1622.7,1622.7,1628.71,1628.71,1628.71,1634.72,1634.72,1634.72,1640.73,1640.73,1640.73,1640.73,1640.73,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1646.74,1652.75,1652.75,1652.75,1658.76,1658.76,1658.76,1664.77,1664.77,1670.78,1676.79,1676.79,1682.8,1688.81,1694.82,1700.83,1706.84,1712.85,1718.86,1724.87,1730.88,1736.89,1742.9,1748.91,1754.92,1760.93,1766.94,1772.95,1778.96,1784.97,1790.98,1796.99,1803,1809.01,1815.02,1821.03,1827.04,1833.05,1839.06,1845.07,1851.08,1857.09,1863.1,1869.11,1875.12,1881.13,1887.14,1893.15,1899.16,1905.17,1911.18,1917.19,1923.2,1929.21,1935.22,1941.23,1947.24,1953.25,1959.26,1965.27,1971.28,1977.29,1983.3,1989.31,1995.32,2001.33,2007.34,2013.35,2019.36,2025.37,2031.38,2037.39,2043.4,2049.41,2055.42,2061.43,2067.44,2073.45,2079.46,2085.47,2091.48,2097.49,2103.5,2109.51,2115.52,2121.53,2127.54,2133.55,2139.56,2145.57,2151.58,2157.59,2163.6,2169.61,2175.62,2181.63,2187.64,2193.65,2199.66,2205.67,2211.68,2217.69,2223.7,2229.71,2235.72,2241.73,2247.74,2253.75,2259.76,2265.77,2271.78,2277.79,2283.8,2289.81,2295.82,2301.83,2307.84,2313.85,2319.86,2325.87,2331.88,2337.89,2343.9,2349.91,2355.92,2361.93,2367.94,2373.95,2379.96,2385.97,2391.98,2397.99,2404,2410.01,2416.02,2422.03,2428.04,2434.05,2440.06,2446.07,2452.08,2458.09,2464.1,2470.11,2476.12,2482.13,2488.14,2494.15,2500.16,2506.17,2512.18,2518.19,2524.2,2530.21,2536.22,2542.23,2548.24,2554.25,2560.26,2566.27,2572.28,2578.29,2584.3,2590.31,2596.32,2602.33,2608.34,2614.35,2620.36,2626.37,2632.38,2638.39,2644.4,2650.41,2656.42,2662.43,2668.44,2674.45,2680.46,2686.47,2692.48,2698.49,2704.5,2710.51,2716.52,2722.53,2728.54,2734.55,2740.56,2746.57,2752.58,2758.59,2764.6,2770.61,2776.62,2782.63,2788.64,2794.65,2800.66,2806.67,2812.68,2818.69,2824.7,2830.71,2836.72,2842.73,2848.74,2854.75,2860.76,2866.77,2872.78,2878.79,2884.8,2890.81,2896.82,2902.83,2908.84,2914.85,2920.86,2926.87,2932.88,2938.89,2944.9,2950.91,2956.92,2962.93,2968.94,2974.95,2980.96,2986.97,2992.98,2998.99,3005,3011.01,3017.02,3023.03,3029.04,3035.05,3041.06,3047.07,3053.08,3059.09,3065.1,3071.11,3077.12,3083.13,3089.14,3095.15,3101.16,3107.17,3113.18,3119.19,3125.2,3131.21,3137.22,3143.23,3149.24,3155.25,3161.26,3167.27,3173.28,3179.29,3185.3,3191.31,3197.32,3203.33,3209.34,3215.35,3221.36,3227.37,3233.38,3239.39,3245.4,3251.41,3251.41,3257.42,3263.43,3269.44,3275.45,3275.45,3281.46,3287.47,3293.48,3293.48,3299.49,3299.49,3305.5,3311.51,3311.51,3317.52,3317.52,3323.53,3323.53,3329.54,3329.54,3335.55,3335.55,3335.55,3341.56,3341.56,3347.57,3347.57,3347.57,3353.58,3353.58,3353.58,3353.58,3359.59,3359.59,3359.59,3365.6,3365.6,3365.6,3365.6,3365.6,3365.6,3371.61,3371.61,3371.61,3371.61,3371.61,3371.61,3371.61,3371.61,3371.61,3371.61,3371.61,3371.61,3371.61,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62,3377.62};
   float course_y[kCourseParamNum] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6.01,6.01,6.01,6.01,12.02,12.02,18.03,18.03,18.03,24.04,30.05,30.05,36.06,42.07,42.07,48.08,54.09,60.1,66.11,72.12,78.13,84.14,90.15,96.16,102.17,108.18,114.19,120.2,126.21,132.22,138.23,144.24,150.25,156.26,162.27,168.28,174.29,180.3,186.31,192.32,198.33,204.34,210.35,216.36,222.37,228.38,234.39,240.4,246.41,252.42,258.43,264.44,270.45,276.46,282.47,288.48,294.49,300.5,306.51,312.52,318.53,324.54,330.55,336.56,342.57,348.58,354.59,360.6,366.61,372.62,378.63,384.64,390.65,396.66,402.67,408.68,414.69,420.7,426.71,432.72,438.73,444.74,450.75,456.76,462.77,468.78,474.79,480.8,486.81,492.82,498.83,504.84,510.85,516.86,522.87,528.88,534.89,540.9,546.91,552.92,558.93,564.94,570.95,576.96,582.97,588.98,594.99,601,607.01,613.02,619.03,625.04,631.05,637.06,643.07,649.08,655.09,661.1,667.11,673.12,679.13,685.14,691.15,697.16,703.17,709.18,715.19,721.2,727.21,733.22,739.23,745.24,751.25,757.26,763.27,769.28,775.29,781.3,787.31,793.32,799.33,805.34,811.35,817.36,823.37,829.38,835.39,841.4,847.41,853.42,859.43,865.44,871.45,877.46,883.47,889.48,895.49,901.5,907.51,913.52,919.53,925.54,931.55,937.56,943.57,949.58,955.59,961.6,967.61,973.62,979.63,985.64,991.65,997.66,1003.67,1009.68,1015.69,1021.7,1027.71,1033.72,1039.73,1045.74,1051.75,1057.76,1063.77,1069.78,1075.79,1081.8,1087.81,1093.82,1099.83,1105.84,1111.85,1117.86,1123.87,1129.88,1135.89,1141.9,1147.91,1153.92,1159.93,1165.94,1171.95,1177.96,1183.97,1189.98,1195.99,1202,1208.01,1214.02,1220.03,1226.04,1232.05,1238.06,1244.07,1250.08,1256.09,1262.1,1268.11,1274.12,1280.13,1286.14,1292.15,1298.16,1304.17,1310.18,1316.19,1322.2,1328.21,1334.22,1340.23,1346.24,1352.25,1358.26,1364.27,1370.28,1376.29,1382.3,1388.31,1394.32,1400.33,1406.34,1412.35,1418.36,1424.37,1430.38,1436.39,1442.4,1448.41,1454.42,1460.43,1466.44,1472.45,1478.46,1484.47,1490.48,1496.49,1502.5,1508.51,1514.52,1520.53,1526.54,1532.55,1538.56,1544.57,1550.58,1556.59,1562.6,1568.61,1574.62,1580.63,1586.64,1592.65,1598.66,1604.67,1610.68,1616.69,1622.7,1628.71,1634.72,1640.73,1646.74,1652.75,1658.76,1664.77,1670.78,1676.79,1682.8,1688.81,1694.82,1700.83,1706.84,1712.85,1718.86,1724.87,1730.88,1736.89,1742.9,1748.91,1754.92,1760.93,1766.94,1772.95,1778.96,1784.97,1790.98,1796.99,1803,1809.01,1815.02,1821.03,1827.04,1833.05,1839.06,1845.07,1851.08,1857.09,1863.1,1869.11,1875.12,1881.13,1887.14,1893.15,1899.16,1905.17,1911.18,1917.19,1923.2,1929.21,1935.22,1941.23,1947.24,1953.25,1959.26,1965.27,1971.28,1977.29,1983.3,1989.31,1995.32,2001.33,2007.34,2013.35,2019.36,2025.37,2031.38,2037.39,2043.4,2049.41,2055.42,2061.43,2067.44,2073.45,2079.46,2085.47,2091.48,2097.49,2103.5,2109.51,2115.52,2121.53,2127.54,2133.55,2139.56,2145.57,2151.58,2157.59,2163.6,2169.61,2175.62,2181.63,2181.63,2187.64,2187.64,2193.65,2193.65,2199.66,2199.66,2199.66,2205.67,2205.67,2205.67,2205.67,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2211.68,2205.67,2205.67,2205.67,2205.67,2205.67,2205.67,2199.66,2199.66,2199.66,2199.66,2199.66,2193.65,2193.65,2193.65,2193.65,2187.64,2187.64,2187.64,2181.63,2181.63,2175.62,2175.62,2175.62,2169.61,2169.61,2163.6,2163.6,2157.59,2157.59,2151.58,2151.58,2145.57,2139.56,2139.56,2133.55,2133.55,2127.54,2121.53,2115.52,2115.52,2109.51,2103.5,2097.49,2091.48,2091.48,2085.47,2079.46,2073.45,2067.44,2061.43,2055.42,2049.41,2043.4,2037.39,2031.38,2025.37,2019.36,2013.35,2007.34,2001.33,1995.32,1989.31,1983.3,1977.29,1971.28,1965.27,1959.26,1953.25,1947.24,1941.23,1935.22,1929.21,1923.2,1917.19,1911.18,1905.17,1899.16,1893.15,1887.14,1881.13,1875.12,1869.11,1863.1,1857.09,1851.08,1845.07,1839.06,1833.05,1827.04,1821.03,1815.02,1809.01,1803,1796.99,1790.98,1784.97,1778.96,1772.95,1766.94,1760.93,1754.92,1748.91,1742.9,1736.89,1730.88,1724.87,1718.86,1712.85,1706.84,1700.83,1694.82,1688.81,1682.8,1676.79,1670.78,1664.77,1658.76,1652.75,1646.74,1640.73,1634.72,1628.71,1622.7,1616.69,1610.68,1604.67,1598.66,1592.65,1586.64,1580.63,1574.62,1568.61,1562.6,1556.59,1550.58,1544.57,1538.56,1532.55,1526.54,1520.53,1514.52,1508.51,1502.5,1496.49,1490.48,1484.47,1478.46,1472.45,1466.44,1460.43,1454.42,1448.41,1442.4,1436.39,1430.38,1424.37,1418.36,1412.35,1406.34,1400.33,1394.32,1388.31,1382.3,1376.29,1370.28,1364.27,1358.26,1352.25,1346.24,1340.23,1334.22,1328.21,1322.2,1316.19,1310.18,1304.17,1298.16,1292.15,1286.14,1280.13,1274.12,1268.11,1262.1,1256.09,1250.08,1244.07,1238.06,1232.05,1226.04,1220.03,1214.02,1208.01,1202,1195.99,1189.98,1183.97,1177.96,1171.95,1165.94,1159.93,1153.92,1147.91,1141.9,1135.89,1129.88,1123.87,1117.86,1111.85,1105.84,1099.83,1093.82,1087.81,1081.8,1075.79,1069.78,1063.77,1057.76,1051.75,1045.74,1039.73,1033.72,1027.71,1021.7,1015.69,1009.68,1003.67,997.66,991.65,985.64,979.63,973.62,967.61,961.6,955.59,949.58,943.57,937.56,931.55,925.54,919.53,913.52,907.51,901.5,895.49,889.48,883.47,877.46,871.45,865.44,859.43,853.42,847.41,841.4,835.39,829.38,823.37,817.36,811.35,805.34,799.33,793.32,787.31,781.3,775.29,769.28,763.27,757.26,751.25,745.24,739.23,733.22,727.21,721.2,715.19,709.18,703.17,697.16,691.15,685.14,679.13,673.12,667.11,661.1,655.09,649.08,643.07,637.06,631.05,625.04,619.03,613.02,607.01,601,594.99,588.98,582.97,576.96,570.95,564.94,558.93,552.92,546.91,540.9,534.89,528.88,522.87,516.86,510.85,504.84,498.83,492.82,486.81,480.8};
  /////////////////////////////追従させる経路////////////////////////////////////////
};

class Localize {
 public:
  Localize(MotorIo* motor_io);
  void Update();
  double distance_ = 0;
  double theta_ = 0;
  double odometry_x = 0;
  double odometry_y = 0;

 private:
  Odometry* odometry_;
  PurePursuit* pure_pursuit_;
  char c[256];
};

#endif  // ETRC22_ETRC_INFO_H_
