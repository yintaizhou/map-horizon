/***************************************************************************
 *
 * Copyright (c) 2014 Baidu.com, Inc. All Rights Reserved
 *
 **************************************************************************/

/**
 * @file mm_tool.h
 * @author huguojing01(com@baidu.com)
 * @date 2014/06/06 16:21:57
 * @brief
 *
 **/

#ifndef  MAPMATCH_MM_TOOL_H
#define  MAPMATCH_MM_TOOL_H

///
/// \breif 求一个点到其在线段上投影点之间的距离以及投影点离首点的距离。输入坐标类型为gcj02
///        同时记录投影点是落在线上还是延长线上面
/// \param p_x, p_y 点的坐标
/// \param l_x1, l_y1 线段首点
/// \param l_x2, l_y2 线段末点
/// \param dist_to_line 点到投影点的距离，单位米
/// \param dist_to_snode 投影点到首点的距离
/// \param project_type 投影点类型 0:投影点在线上，1:投影点在起点之前，2:投影点在末点之后
///
void calculate_projection(double p_x, double p_y,
        double l_x1, double l_y1,
        double l_x2, double l_y2,
        double& dist_to_line,
        double& dist_to_snode,
        int& project_type);


#endif 
