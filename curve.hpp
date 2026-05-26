//T形曲线改，输入值为（最大加速度，最大速度，目标位置）
//tc_指运动到目标完成时间；
//tm_指以最大速度运动所需时间；
//v_为最大运动速度；
//a_为最大运动加速度；
//ta_指加速时间；
//p_指最终运动位置；
//pa_指加减速所需要运动的位置。
class TCurve2
{
private:
    double tc_;
    double tm_;
    double v_;
    double a_;
    double ta_;
    double p_;
    double pa_;

public:
    auto getTCurve(int count) -> double;
    auto getCurveParam() -> void;
    auto getTc() -> double { return tc_; };
    TCurve2(double a, double v, double p) { a_ = a; v_ = v; p_ = p; }
    ~TCurve2() {}
};