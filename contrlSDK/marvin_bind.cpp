/*
 * marvin_bind.cpp
 * pybind11 bindings for Marvin Robot Control SDK
 *
 * Build:
 *   make -f Makefile_pybind
 *
 * Usage:
 *   import sys
 *   sys.path.insert(0, '/path/to/contrlSDK')
 *   import marvin_sdk as sdk
 *
 *   sdk.connect('192.168.1.190')
 *   dcss = sdk.DCSS()
 *   sdk.get_buf(dcss)
 *   print(dcss.outputs[0].fb_joint_pos)
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <string>
#include <vector>
#include <stdexcept>
#include <cstring>
#include <algorithm>

// SDK headers — compile everything together, no dlopen needed
#define CMPL_LIN
#include "FxRtCSDef.h"   // DCSS, RT_IN, RT_OUT, StateCtr, ArmState
#include "PointSet.h"    // CPointSet
#include "MarvinSDK.h"   // extern "C" API
#include "Robot.h"       // CRobot static methods (for CPointSet* overloads)

namespace py = pybind11;

// ─── Helpers ─────────────────────────────────────────────────────────────────

static void check_arm(const std::string& arm)
{
    if (arm != "A" && arm != "B")
        throw py::value_error("arm must be 'A' or 'B', got '" + arm + "'");
}

// Accept py::array_t<double> or a plain Python list; validate size; return ptr.
static double* require_doubles(py::array_t<double, py::array::c_style>& arr,
                               py::ssize_t expected, const char* name)
{
    auto buf = arr.request();
    if (buf.size < expected)
        throw py::value_error(std::string(name) + " must have at least " +
                              std::to_string(expected) + " elements (got " +
                              std::to_string(buf.size) + ")");
    return static_cast<double*>(buf.ptr);
}

// ─── Module ──────────────────────────────────────────────────────────────────

PYBIND11_MODULE(marvin_sdk, m)
{
    m.doc() =
        "Marvin Robot Control SDK — pybind11 bindings\n"
        "\n"
        "Quick-start:\n"
        "    import marvin_sdk as sdk\n"
        "    import numpy as np\n"
        "\n"
        "    sdk.connect('192.168.1.190')\n"
        "    import time; time.sleep(0.5)\n"
        "\n"
        "    dcss = sdk.DCSS()\n"
        "    sdk.get_buf(dcss)\n"
        "    print(dcss.outputs[0].fb_joint_pos)\n"
        "\n"
        "    # Position mode\n"
        "    sdk.clear_set()\n"
        "    sdk.clear_err('A')\n"
        "    sdk.set_vel_acc('A', 10, 10)\n"
        "    sdk.set_state('A', sdk.ArmState.POSITION)\n"
        "    sdk.set_joint_cmd('A', np.zeros(7))\n"
        "    sdk.send_cmd()\n"
        "\n"
        "    sdk.release()\n";

    // =========================================================================
    // Enums
    // =========================================================================

    py::enum_<ArmState>(m, "ArmState",
        "机械臂状态码，用于 set_state() 的 state 参数和订阅数据 cur_state 字段")
        .value("IDLE",              ARM_STATE_IDLE,              "下伺服")
        .value("POSITION",          ARM_STATE_POSITION,          "位置跟随模式")
        .value("PVT",               ARM_STATE_PVT,               "PVT 离线轨迹模式")
        .value("TORQ",              ARM_STATE_TORQ,              "扭矩/阻抗模式")
        .value("RELEASE",           ARM_STATE_RELEASE,           "协作释放模式")
        .value("ERROR",             ARM_STATE_ERROR,             "报错，需清错")
        .value("TRANS_TO_POSITION", ARM_STATE_TRANS_TO_POSITION, "切换中（持续则切换失败）")
        .value("TRANS_TO_PVT",      ARM_STATE_TRANS_TO_PVT,      "切换中")
        .value("TRANS_TO_TORQ",     ARM_STATE_TRANS_TO_TORQ,     "切换中")
        .value("TRANS_TO_RELEASE",  ARM_STATE_TRANS_TO_RELEASE,  "切换中")
        .value("TRANS_TO_IDLE",     ARM_STATE_TRANS_TO_IDLE,     "切换中")
        .export_values();

    // =========================================================================
    // Data structures
    // =========================================================================

    // ── StateCtr ──────────────────────────────────────────────────────────────
    py::class_<StateCtr>(m, "StateCtr",
        "单臂状态控制信息（来自 DCSS.states[i]）")
        .def(py::init<>())
        .def_readwrite("cur_state", &StateCtr::m_CurState,
            "当前状态，对照 ArmState 枚举")
        .def_readwrite("cmd_state", &StateCtr::m_CmdState,
            "指令状态")
        .def_readwrite("err_code",  &StateCtr::m_ERRCode,
            "错误码，非 0 时调用 clear_err()");

    // ── RT_OUT ────────────────────────────────────────────────────────────────
    py::class_<RT_OUT>(m, "RtOut",
        "单臂实时反馈数据（来自 DCSS.outputs[i]）\n"
        "索引 0=左臂A，1=右臂B")
        .def(py::init<>())
        .def_readwrite("frame_serial",   &RT_OUT::m_OutFrameSerial,
            "输出帧序号 0~1000000 取模")
        .def_readwrite("tip_di",         &RT_OUT::m_TipDI,
            "末端按钮是否按下")
        .def_readwrite("low_speed_flag", &RT_OUT::m_LowSpdFlag,
            "机器人低速/停止标志，=1 表示各关节速度 <0.5°/s，可用于判断是否到位")
        .def_readwrite("traj_state",     &RT_OUT::m_TrajState,
            "规划状态: 0=无 1=接收中 2=已接收 ≥3=执行中")
        // Arrays as read-only properties returning Python lists
        .def_property_readonly("fb_joint_pos",
            [](const RT_OUT& s){
                return std::vector<float>(s.m_FB_Joint_Pos, s.m_FB_Joint_Pos+7);},
            "反馈关节位置 list[7]，单位度")
        .def_property_readonly("fb_joint_vel",
            [](const RT_OUT& s){
                return std::vector<float>(s.m_FB_Joint_Vel, s.m_FB_Joint_Vel+7);},
            "反馈关节速度 list[7]，单位度/s")
        .def_property_readonly("fb_joint_posE",
            [](const RT_OUT& s){
                return std::vector<float>(s.m_FB_Joint_PosE, s.m_FB_Joint_PosE+7);},
            "反馈关节外编位置 list[7]")
        .def_property_readonly("fb_joint_cmd",
            [](const RT_OUT& s){
                return std::vector<float>(s.m_FB_Joint_Cmd, s.m_FB_Joint_Cmd+7);},
            "位置关节指令（回显）list[7]")
        .def_property_readonly("fb_joint_ctoq",
            [](const RT_OUT& s){
                return std::vector<float>(s.m_FB_Joint_CToq, s.m_FB_Joint_CToq+7);},
            "反馈关节电流（千分比）list[7]")
        .def_property_readonly("fb_joint_stoq",
            [](const RT_OUT& s){
                return std::vector<float>(s.m_FB_Joint_SToq, s.m_FB_Joint_SToq+7);},
            "反馈关节传感器扭矩 list[7]，单位 N·m")
        .def_property_readonly("fb_joint_them",
            [](const RT_OUT& s){
                return std::vector<float>(s.m_FB_Joint_Them, s.m_FB_Joint_Them+7);},
            "反馈关节温度 list[7]")
        .def_property_readonly("est_joint_firc",
            [](const RT_OUT& s){
                return std::vector<float>(s.m_EST_Joint_Firc, s.m_EST_Joint_Firc+7);},
            "关节摩擦力估计值 list[7]")
        .def_property_readonly("est_joint_firc_dot",
            [](const RT_OUT& s){
                return std::vector<float>(s.m_EST_Joint_Firc_Dot,
                                         s.m_EST_Joint_Firc_Dot+7);},
            "关节力扰动估计值微分 list[7]")
        .def_property_readonly("est_joint_force",
            [](const RT_OUT& s){
                return std::vector<float>(s.m_EST_Joint_Force,
                                         s.m_EST_Joint_Force+7);},
            "关节外力估计值 list[7]（v1003_37+支持任意状态）")
        .def_property_readonly("est_cart_fn",
            [](const RT_OUT& s){
                return std::vector<float>(s.m_EST_Cart_FN, s.m_EST_Cart_FN+6);},
            "末端扰动估计值 list[6]（Fx Fy Fz Tx Ty Tz）");

    // ── RT_IN ─────────────────────────────────────────────────────────────────
    py::class_<RT_IN>(m, "RtIn",
        "单臂实时输入状态镜像（来自 DCSS.inputs[i]）")
        .def(py::init<>())
        .def_readwrite("rt_in_switch",         &RT_IN::m_RtInSwitch)
        .def_readwrite("imp_type",             &RT_IN::m_ImpType,
            "当前阻抗类型: 1=关节 2=笛卡尔 3=力控")
        .def_readwrite("in_frame_serial",      &RT_IN::m_InFrameSerial)
        .def_readwrite("frame_miss_cnt",       &RT_IN::m_FrameMissCnt,
            "丢帧计数")
        .def_readwrite("max_frame_miss_cnt",   &RT_IN::m_MaxFrameMissCnt)
        .def_readwrite("sys_cyc",              &RT_IN::m_SysCyc)
        .def_readwrite("sys_cyc_miss_cnt",     &RT_IN::m_SysCycMissCnt)
        .def_readwrite("max_sys_cyc_miss_cnt", &RT_IN::m_MaxSysCycMissCnt)
        .def_readwrite("joint_vel_ratio",      &RT_IN::m_Joint_Vel_Ratio,
            "关节速度限制百分比")
        .def_readwrite("joint_acc_ratio",      &RT_IN::m_Joint_Acc_Ratio,
            "关节加速度限制百分比")
        .def_readwrite("drag_sp_type",         &RT_IN::m_DragSpType)
        .def_readwrite("cart_kd_type",         &RT_IN::m_Cart_KD_Type)
        .def_readwrite("cart_kn",              &RT_IN::m_Cart_KN)
        .def_readwrite("cart_dn",              &RT_IN::m_Cart_DN)
        .def_readwrite("force_fb_type",        &RT_IN::m_Force_FB_Type)
        .def_readwrite("force_type",           &RT_IN::m_Force_Type)
        .def_readwrite("force_adj_lmt",        &RT_IN::m_Force_AdjLmt)
        .def_readwrite("force_cmd",            &RT_IN::m_Force_Cmd)
        .def_readwrite("pvt_id",               &RT_IN::m_PvtID)
        .def_readwrite("pvt_run_id",           &RT_IN::m_Pvt_RunID,
            "PVT 运行 ID: 0=无 1-249=用户 250~252=标定")
        .def_readwrite("pvt_run_state",        &RT_IN::m_Pvt_RunState,
            "PVT 状态: 0=空闲 1=加载 2=运行 3=错误")
        .def_property_readonly("tool_kine",
            [](const RT_IN& s){
                return std::vector<float>(s.m_ToolKine, s.m_ToolKine+6);},
            "工具运动学参数 list[6]")
        .def_property_readonly("tool_dyn",
            [](const RT_IN& s){
                return std::vector<float>(s.m_ToolDyn, s.m_ToolDyn+10);},
            "工具动力学参数 list[10]")
        .def_property_readonly("joint_cmd_pos",
            [](const RT_IN& s){
                return std::vector<float>(s.m_Joint_CMD_Pos, s.m_Joint_CMD_Pos+7);},
            "关节位置指令回显 list[7]")
        .def_property_readonly("joint_k",
            [](const RT_IN& s){
                return std::vector<float>(s.m_Joint_K, s.m_Joint_K+7);},
            "关节阻抗刚度 list[7]")
        .def_property_readonly("joint_d",
            [](const RT_IN& s){
                return std::vector<float>(s.m_Joint_D, s.m_Joint_D+7);},
            "关节阻抗阻尼 list[7]")
        .def_property_readonly("drag_sp_para",
            [](const RT_IN& s){
                return std::vector<float>(s.m_DragSpPara, s.m_DragSpPara+6);},
            "拖动参数 list[6]")
        .def_property_readonly("cart_k",
            [](const RT_IN& s){
                return std::vector<float>(s.m_Cart_K, s.m_Cart_K+6);},
            "笛卡尔阻抗刚度 list[6]")
        .def_property_readonly("cart_d",
            [](const RT_IN& s){
                return std::vector<float>(s.m_Cart_D, s.m_Cart_D+6);},
            "笛卡尔阻抗阻尼 list[6]")
        .def_property_readonly("force_dir",
            [](const RT_IN& s){
                return std::vector<float>(s.m_Force_Dir, s.m_Force_Dir+6);},
            "力控方向 list[6]")
        .def_property_readonly("force_pidul",
            [](const RT_IN& s){
                return std::vector<float>(s.m_Force_PIDUL, s.m_Force_PIDUL+7);},
            "力控 PID 参数 list[7]");

    // ── DCSS ─────────────────────────────────────────────────────────────────
    py::class_<DCSS>(m, "DCSS",
        "订阅数据总包\n"
        "  dcss = sdk.DCSS()\n"
        "  sdk.get_buf(dcss)          # 高频推荐：原地填充\n"
        "  dcss.states[0].cur_state   # 左臂A 当前状态\n"
        "  dcss.outputs[0].fb_joint_pos  # 左臂A 关节位置")
        .def(py::init<>())
        .def_property_readonly("states",
            [](const DCSS& d){
                return std::vector<StateCtr>(d.m_State, d.m_State+2);},
            "list[StateCtr x2]: [0]=左臂A [1]=右臂B 状态控制信息")
        .def_property_readonly("outputs",
            [](const DCSS& d){
                return std::vector<RT_OUT>(d.m_Out, d.m_Out+2);},
            "list[RtOut x2]: [0]=左臂A [1]=右臂B 反馈数据")
        .def_property_readonly("inputs",
            [](const DCSS& d){
                return std::vector<RT_IN>(d.m_In, d.m_In+2);},
            "list[RtIn x2]: [0]=左臂A [1]=右臂B 指令镜像");

    // ── CPointSet ────────────────────────────────────────────────────────────
    py::class_<CPointSet>(m, "CPointSet",
        "轨迹点集，用于笛卡尔空间直线规划 (set_pln_cart)\n"
        "\n"
        "用法:\n"
        "    pset = sdk.CPointSet()\n"
        "    pset.init(7)                      # 7维（关节数）\n"
        "    pset.set_point(np.array([...]))    # 追加一个点\n"
        "    sdk.set_pln_cart('A', pset)")
        .def(py::init<>())
        .def("init",
            [](CPointSet& s, int ptype){
                return s.OnInit(static_cast<PoinType>(ptype));},
            py::arg("ptype"),
            "初始化维度。ptype 为维度数 (1~40)，7=7关节")
        .def("init_with_cap",
            [](CPointSet& s, int ptype, long cap){
                return s.OnInit(static_cast<PoinType>(ptype), cap);},
            py::arg("ptype"), py::arg("cap"),
            "初始化维度并预分配容量")
        .def("empty",
            &CPointSet::OnEmpty,
            "清空点集")
        .def("get_type",
            [](CPointSet& s){ return static_cast<int>(s.OnGetType()); },
            "返回当前维度数 (int)")
        .def("get_point_num",
            &CPointSet::OnGetPointNum,
            "返回点的数量")
        .def("set_point",
            [](CPointSet& s, py::array_t<double, py::array::c_style> v){
                auto buf = v.request();
                return s.OnSetPoint(static_cast<double*>(buf.ptr));},
            py::arg("point"),
            "追加一个点 (numpy array 或 list，长度=维度数)")
        .def("get_point",
            [](CPointSet& s, long pos) -> py::object {
                double* p = s.OnGetPoint(pos);
                if (!p) return py::none();
                int dim = static_cast<int>(s.OnGetType());
                return py::cast(std::vector<double>(p, p+dim));},
            py::arg("pos"),
            "读取第 pos 个点，返回 list[float] 或 None")
        .def("save",
            [](CPointSet& s, const std::string& path){
                return s.OnSave(const_cast<char*>(path.c_str()));},
            py::arg("path"), "保存点集到文件")
        .def("save_csv",
            [](CPointSet& s, const std::string& path){
                return s.OnSaveCSV(const_cast<char*>(path.c_str()));},
            py::arg("path"), "保存点集为 CSV")
        .def("load",
            [](CPointSet& s, const std::string& path){
                return s.OnLoad(const_cast<char*>(path.c_str()));},
            py::arg("path"), "从文件加载点集")
        .def("load_fast",
            [](CPointSet& s, const std::string& path){
                return s.OnLoadFast(const_cast<char*>(path.c_str()));},
            py::arg("path"), "快速加载点集")
        .def("set_tag",
            [](CPointSet& s, const std::string& tag){
                return s.OnSetTag(const_cast<char*>(tag.c_str()));},
            py::arg("tag"))
        .def("get_tag",
            [](CPointSet& s) -> std::string {
                char* t = s.OnGetTag();
                return t ? std::string(t) : "";});

    // =========================================================================
    // Connection
    // =========================================================================

    m.def("connect",
        [](const std::string& ip) {
            unsigned char a,b,c,d;
            if (std::sscanf(ip.c_str(), "%hhu.%hhu.%hhu.%hhu", &a,&b,&c,&d) != 4)
                throw py::value_error("invalid IP address: '" + ip + "'");
            return static_cast<bool>(OnLinkTo(a,b,c,d));},
        py::arg("ip"),
        "连接机器人\n"
        "  connect('192.168.1.190')\n"
        "  连接后 sleep(0.5) 再清错");

    m.def("release",
        []{ return static_cast<bool>(OnRelease()); },
        "释放机器人连接，其他进程才能接管");

    m.def("sdk_version",
        []{ return OnGetSDKVersion(); },
        "返回 SDK 大版本号 (long)");

    // =========================================================================
    // Subscribe
    // =========================================================================

    m.def("get_buf",
        [](DCSS& dcss){
            return static_cast<bool>(OnGetBuf(&dcss));},
        py::arg("dcss"),
        "原地填充 DCSS 对象（推荐高频调用，避免每次分配）\n"
        "  dcss = sdk.DCSS()\n"
        "  while True:\n"
        "      sdk.get_buf(dcss)\n"
        "      pos = dcss.outputs[0].fb_joint_pos");

    m.def("subscribe",
        []{
            DCSS d;
            OnGetBuf(&d);
            return d;},
        "订阅并返回新 DCSS 对象（低频/调试用）");

    // =========================================================================
    // Emergency / Reset
    // =========================================================================

    m.def("emg",
        [](const std::string& arm){
            if (arm=="A")       OnEMG_A();
            else if (arm=="B")  OnEMG_B();
            else if (arm=="AB") OnEMG_AB();
            else throw py::value_error("arm must be 'A', 'B' or 'AB'");},
        py::arg("arm"),
        "软急停。急停后自动下伺服，需 clear_err → set_state(POSITION/TORQ) 恢复");

    m.def("servo_reset",
        [](const std::string& arm, int axis){
            check_arm(arm);
            if (arm=="A") OnServoReset_A(axis);
            else          OnServoReset_B(axis);},
        py::arg("arm"), py::arg("axis"),
        "指定轴伺服软复位 (axis: 0-6)");

    // =========================================================================
    // Error
    // =========================================================================

    m.def("get_servo_err",
        [](const std::string& arm){
            check_arm(arm);
            long codes[7] = {};
            if (arm=="A") OnGetServoErr_A(codes);
            else          OnGetServoErr_B(codes);
            return std::vector<long>(codes, codes+7);},
        py::arg("arm"),
        "获取 7 轴伺服错误码 list[long]（十进制，转十六进制对照手册）");

    m.def("clear_err",
        [](const std::string& arm){
            check_arm(arm);
            if (arm=="A") OnClearErr_A();
            else          OnClearErr_B();},
        py::arg("arm"),
        "清除伺服错误。建议 connect 后 sleep(0.5) 再调用");

    // =========================================================================
    // Log
    // =========================================================================

    m.def("log_on",        []{ OnLogOn(); },       "开启全局日志");
    m.def("log_off",       []{ OnLogOff(); },      "关闭全局日志");
    m.def("local_log_on",  []{ OnLocalLogOn(); },  "开启本地日志");
    m.def("local_log_off", []{ OnLocalLogOff(); }, "关闭本地日志");

    // =========================================================================
    // File / System
    // =========================================================================

    m.def("update_system",
        [](const std::string& path){
            return static_cast<bool>(OnUpdateSystem(const_cast<char*>(path.c_str())));},
        py::arg("path"), "升级控制器系统，传入本地升级包路径");

    m.def("download_log",
        [](const std::string& path){
            return static_cast<bool>(OnDownloadLog(const_cast<char*>(path.c_str())));},
        py::arg("path"), "下载控制器日志到本地路径");

    m.def("send_file",
        [](const std::string& local, const std::string& remote){
            return static_cast<bool>(
                OnSendFile(const_cast<char*>(local.c_str()),
                           const_cast<char*>(remote.c_str())));},
        py::arg("local"), py::arg("remote"),
        "上传本地文件到控制器（绝对路径）");

    m.def("recv_file",
        [](const std::string& local, const std::string& remote){
            return static_cast<bool>(
                OnRecvFile(const_cast<char*>(local.c_str()),
                           const_cast<char*>(remote.c_str())));},
        py::arg("local"), py::arg("remote"),
        "从控制器下载文件到本地（绝对路径）");

    m.def("send_pvt",
        [](const std::string& arm, const std::string& path, long serial){
            check_arm(arm);
            if (arm=="A") return static_cast<bool>(
                OnSendPVT_A(const_cast<char*>(path.c_str()), serial));
            else          return static_cast<bool>(
                OnSendPVT_B(const_cast<char*>(path.c_str()), serial));},
        py::arg("arm"), py::arg("path"), py::arg("serial"),
        "上传本地 PVT 文件并存为指定 ID (1-249)");

    // =========================================================================
    // Parameters (get/set/save)
    // =========================================================================

    m.def("get_param_int",
        [](const std::string& name){
            char buf[30] = {};
            std::strncpy(buf, name.c_str(), 29);
            long val = 0;
            long ret = OnGetIntPara(buf, &val);
            return py::make_tuple(ret, val);},
        py::arg("name"),
        "获取整型参数，返回 (ret_code, value)\n"
        "  ret, ver = sdk.get_param_int('VERSION')");

    m.def("set_param_int",
        [](const std::string& name, long val){
            char buf[30] = {};
            std::strncpy(buf, name.c_str(), 29);
            return OnSetIntPara(buf, val);},
        py::arg("name"), py::arg("value"),
        "设置整型参数");

    m.def("get_param_float",
        [](const std::string& name){
            char buf[30] = {};
            std::strncpy(buf, name.c_str(), 29);
            double val = 0;
            long ret = OnGetFloatPara(buf, &val);
            return py::make_tuple(ret, val);},
        py::arg("name"),
        "获取浮点参数，返回 (ret_code, value)");

    m.def("set_param_float",
        [](const std::string& name, double val){
            char buf[30] = {};
            std::strncpy(buf, name.c_str(), 29);
            return OnSetFloatPara(buf, val);},
        py::arg("name"), py::arg("value"),
        "设置浮点参数");

    m.def("save_param",
        []{ return OnSavePara(); },
        "保存参数到配置文件（调用后 sleep(1s) 留够写文件时间）");

    // =========================================================================
    // Data Collection
    // =========================================================================

    m.def("start_gather",
        [](long num, std::vector<long> ids, long rows){
            ids.resize(35, 0);  // SDK requires exactly 35 slots
            return static_cast<bool>(OnStartGather(num, ids.data(), rows));},
        py::arg("num"), py::arg("ids"), py::arg("rows"),
        "开始采集数据\n"
        "  num: 采集列数 (≤35)\n"
        "  ids: 特征序号列表，自动补零到 35 个\n"
        "  rows: 采集行数 (≤1000000)");

    m.def("stop_gather",
        []{ return static_cast<bool>(OnStopGather()); },
        "停止数据采集（等 1ms 后生效）");

    m.def("save_gather",
        [](const std::string& path){
            return static_cast<bool>(
                OnSaveGatherData(const_cast<char*>(path.c_str())));},
        py::arg("path"),
        "保存采集数据到文件（调用后 sleep(0.2s)）");

    m.def("save_gather_csv",
        [](const std::string& path){
            return static_cast<bool>(
                OnSaveGatherDataCSV(const_cast<char*>(path.c_str())));},
        py::arg("path"),
        "保存采集数据为 CSV");

    // =========================================================================
    // Command frame  (MUST be enclosed by clear_set() … send_cmd())
    // =========================================================================

    m.def("clear_set",
        []{ return static_cast<bool>(OnClearSet()); },
        "【指令帧开始】清除发送缓存，后续设置类 API 必须在本函数之后调用");

    m.def("send_cmd",
        []{ return static_cast<bool>(OnSetSend()); },
        "【指令帧结束】将帧内所有指令发送给机器人");

    m.def("send_cmd_wait",
        [](long timeout_ms){
            return OnSetSendWaitResponse(timeout_ms);},
        py::arg("timeout_ms"),
        "发送并等待响应，返回实际延时 ms；0=超时，-1=错误（建议 50-100ms）");

    // =========================================================================
    // Mode / State (inside clear_set...send_cmd)
    // =========================================================================

    m.def("set_state",
        [](const std::string& arm, int state){
            check_arm(arm);
            if (arm=="A") return static_cast<bool>(OnSetTargetState_A(state));
            else          return static_cast<bool>(OnSetTargetState_B(state));},
        py::arg("arm"), py::arg("state"),
        "切换控制模式\n"
        "  0=下使能  1=位置  2=PVT  3=扭矩  4=协作释放\n"
        "  切换不同大类时建议先切 0 再切目标");

    m.def("set_imp_type",
        [](const std::string& arm, int type){
            check_arm(arm);
            if (arm=="A") return static_cast<bool>(OnSetImpType_A(type));
            else          return static_cast<bool>(OnSetImpType_B(type));},
        py::arg("arm"), py::arg("type"),
        "设置阻抗类型（需在扭矩模式下）\n"
        "  1=关节阻抗  2=笛卡尔阻抗  3=力控");

    m.def("set_drag",
        [](const std::string& arm, int dtype){
            check_arm(arm);
            if (arm=="A") return static_cast<bool>(OnSetDragSpace_A(dtype));
            else          return static_cast<bool>(OnSetDragSpace_B(dtype));},
        py::arg("arm"), py::arg("dtype"),
        "设置拖动类型\n"
        "  0=退出拖动  1=关节拖动\n"
        "  2=笛卡尔X  3=笛卡尔Y  4=笛卡尔Z  5=旋转");

    // =========================================================================
    // Parameter setters (inside clear_set...send_cmd)
    // =========================================================================

    m.def("set_vel_acc",
        [](const std::string& arm, int vel, int acc){
            check_arm(arm);
            if (arm=="A") return static_cast<bool>(OnSetJointLmt_A(vel, acc));
            else          return static_cast<bool>(OnSetJointLmt_B(vel, acc));},
        py::arg("arm"), py::arg("vel_ratio"), py::arg("acc_ratio"),
        "设置速度和加速度百分比 (0-100)，PVT 和拖动模式不受此限制");

    m.def("set_tool",
        [](const std::string& arm,
           py::array_t<double, py::array::c_style> kine,
           py::array_t<double, py::array::c_style> dyn){
            check_arm(arm);
            auto* k = require_doubles(kine, 6, "kine");
            auto* d = require_doubles(dyn, 10, "dyn");
            if (arm=="A") return static_cast<bool>(OnSetTool_A(k, d));
            else          return static_cast<bool>(OnSetTool_B(k, d));},
        py::arg("arm"), py::arg("kine"), py::arg("dyn"),
        "设置末端工具参数\n"
        "  kine[6]: XYZABC 运动学参数（mm/度）\n"
        "  dyn[10]: M mx my mz Ixx Ixy Ixz Iyy Iyz Izz 动力学参数");

    m.def("set_joint_kd",
        [](const std::string& arm,
           py::array_t<double, py::array::c_style> K,
           py::array_t<double, py::array::c_style> D){
            check_arm(arm);
            auto* k = require_doubles(K, 7, "K");
            auto* d = require_doubles(D, 7, "D");
            if (arm=="A") return static_cast<bool>(OnSetJointKD_A(k, d));
            else          return static_cast<bool>(OnSetJointKD_B(k, d));},
        py::arg("arm"), py::arg("K"), py::arg("D"),
        "设置关节阻抗刚度 K[7]（N·m/°）和阻尼 D[7]（N·m·s/°）\n"
        "  建议: 1-3轴 K≤2, 4-7轴 K≤1; D 在 0-1 之间");

    m.def("set_cart_kd",
        [](const std::string& arm,
           py::array_t<double, py::array::c_style> K,
           py::array_t<double, py::array::c_style> D,
           int type){
            check_arm(arm);
            auto* k = require_doubles(K, 7, "K");
            auto* d = require_doubles(D, 7, "D");
            if (arm=="A") return static_cast<bool>(OnSetCartKD_A(k, d, type));
            else          return static_cast<bool>(OnSetCartKD_B(k, d, type));},
        py::arg("arm"), py::arg("K"), py::arg("D"), py::arg("type"),
        "设置笛卡尔阻抗参数\n"
        "  K[7]: [Kx Ky Kz Krx Kry Krz Kn]  平移≤3000 N/m, 旋转≤100 N·m/rad\n"
        "  D[7]: 阻尼比例系数 0-1\n"
        "  type: 与 set_imp_type 一致");

    m.def("set_eef_rot",
        [](const std::string& arm, int fc_type,
           py::array_t<double, py::array::c_style> para){
            check_arm(arm);
            auto* p = require_doubles(para, 7, "para");
            if (arm=="A") return static_cast<bool>(OnSetEefRot_A(fc_type, p));
            else          return static_cast<bool>(OnSetEefRot_B(fc_type, p));},
        py::arg("arm"), py::arg("fc_type"), py::arg("para"),
        "设置末端力控类型和笛卡尔方向旋转\n"
        "  fc_type=1: 用户自定义旋转\n"
        "  para[7]: 前3位为基座 XYZ 旋转，后4位填0");

    m.def("set_force_ctrl",
        [](const std::string& arm, int fc_type,
           py::array_t<double, py::array::c_style> fxdir,
           py::array_t<double, py::array::c_style> para,
           double adj_lmt){
            check_arm(arm);
            auto* fd = require_doubles(fxdir, 6, "fxdir");
            auto* fp = require_doubles(para,  7, "para");
            if (arm=="A") return static_cast<bool>(
                OnSetForceCtrPara_A(fc_type, fd, fp, adj_lmt));
            else          return static_cast<bool>(
                OnSetForceCtrPara_B(fc_type, fd, fp, adj_lmt));},
        py::arg("arm"), py::arg("fc_type"),
        py::arg("fxdir"), py::arg("para"), py::arg("adj_lmt"),
        "设置力控参数\n"
        "  fc_type: 0=坐标空间 1=工具空间\n"
        "  fxdir[6]: 力控方向，控制轴置1（如Z: [0,0,1,0,0,0]）\n"
        "  para[7]: 控制参数（目前全0）\n"
        "  adj_lmt: 允许调节范围 (mm)");

    m.def("set_force_cmd",
        [](const std::string& arm, double force){
            check_arm(arm);
            if (arm=="A") return static_cast<bool>(OnSetForceCmd_A(force));
            else          return static_cast<bool>(OnSetForceCmd_B(force));},
        py::arg("arm"), py::arg("force"),
        "设置力控目标 (N 或 N·m)");

    // =========================================================================
    // Motion commands (inside clear_set...send_cmd)
    // =========================================================================

    m.def("set_joint_cmd",
        [](const std::string& arm,
           py::array_t<double, py::array::c_style> joints){
            check_arm(arm);
            auto* j = require_doubles(joints, 7, "joints");
            if (arm=="A") return static_cast<bool>(OnSetJointCmdPos_A(j));
            else          return static_cast<bool>(OnSetJointCmdPos_B(j));},
        py::arg("arm"), py::arg("joints"),
        "设置目标关节角度 joints[7]（度），位置模式和扭矩模式均有效");

    m.def("set_pvt",
        [](const std::string& arm, int id){
            check_arm(arm);
            if (arm=="A") return static_cast<bool>(OnSetPVT_A(id));
            else          return static_cast<bool>(OnSetPVT_B(id));},
        py::arg("arm"), py::arg("id"),
        "选择并运行 PVT 轨迹 (id: 1-249)，需先 send_pvt 上传文件");

    // =========================================================================
    // PLN — joint space planning (anti-jitter)
    // =========================================================================

    m.def("pln_init",
        [](const std::string& path){
            return static_cast<bool>(
                OnInitPlnLmt(const_cast<char*>(path.c_str())));},
        py::arg("path"),
        "初始化关节规划器，传入对应机型的 .MvKDCfg 配置文件路径\n"
        "  pln_init('ccs_m3.MvKDCfg')");

    m.def("set_pln_joint",
        [](const std::string& arm,
           py::array_t<double, py::array::c_style> start,
           py::array_t<double, py::array::c_style> stop,
           double vel, double acc){
            check_arm(arm);
            auto* s = require_doubles(start, 7, "start");
            auto* e = require_doubles(stop,  7, "stop");
            if (arm=="A") return static_cast<bool>(OnSetPlnJoint_A(s, e, vel, acc));
            else          return static_cast<bool>(OnSetPlnJoint_B(s, e, vel, acc));},
        py::arg("arm"), py::arg("start"), py::arg("stop"),
        py::arg("vel"), py::arg("acc"),
        "位置模式关节空间规划指令（防抖动）\n"
        "  start/stop[7]: 起始/目标关节角度（度）\n"
        "  vel/acc: 规划速度/加速度百分比 (0~1)");

    // PLN — Cartesian space planning (straight line)
    m.def("set_pln_cart",
        [](const std::string& arm, CPointSet& pset){
            check_arm(arm);
            // Use CRobot static method to pass CPointSet* directly
            if (arm=="A") return static_cast<bool>(CRobot::OnSetPlnCart_A(&pset));
            else          return static_cast<bool>(CRobot::OnSetPlnCart_B(&pset));},
        py::arg("arm"), py::arg("pset"),
        "执行笛卡尔空间直线规划（需先用 kine SDK 生成 pset）");

    // =========================================================================
    // End-effector communication (485/CAN)
    // =========================================================================

    m.def("clear_ch_data",
        [](const std::string& arm){
            check_arm(arm);
            if (arm=="A") return static_cast<bool>(OnClearChDataA());
            else          return static_cast<bool>(OnClearChDataB());},
        py::arg("arm"),
        "清除末端模组接收缓存");

    m.def("set_ch_data",
        [](const std::string& arm, py::bytes data_bytes, long ch){
            check_arm(arm);
            std::string s = static_cast<std::string>(data_bytes);
            if (s.size() > 256)
                throw py::value_error("data exceeds 256 bytes");
            unsigned char buf[256] = {};
            std::memcpy(buf, s.data(), s.size());
            long sz = static_cast<long>(s.size());
            if (arm=="A") return static_cast<bool>(OnSetChDataA(buf, sz, ch));
            else          return static_cast<bool>(OnSetChDataB(buf, sz, ch));},
        py::arg("arm"), py::arg("data"), py::arg("ch"),
        "发送数据到末端模组，每包 ≤256 字节\n"
        "  ch: 1=CANFD  2=COM1  3=COM2\n"
        "  data: bytes 或 bytearray");

    m.def("get_ch_data",
        [](const std::string& arm){
            check_arm(arm);
            unsigned char buf[256] = {};
            long ch = 0;
            long size = 0;
            if (arm=="A") size = OnGetChDataA(buf, &ch);
            else          size = OnGetChDataB(buf, &ch);
            size_t n = static_cast<size_t>(std::max(0L, size));
            return py::make_tuple(py::bytes(reinterpret_cast<char*>(buf), n), ch);},
        py::arg("arm"),
        "读取末端模组返回数据，返回 (bytes, channel)");
}
