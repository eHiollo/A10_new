// #ifndef SKILL_FOLLOW_HPP
// #define SKILL_FOLLOW_HPP

// #include "fw/motion.hpp"
// #include "fw/skill.hpp"
// #include <vector>

// namespace rtb::skill {

// class MultiAxisFollowSkill : public fw::SkillBase<MultiAxisFollowSkill> {
// public:
//     struct Port {
//         struct Config {
//             std::vector<double> target_pos;
//             double v_max = 0.5;
//             double a_max = 5.0;
//             double dt = 0.004;
//             double tolerance = 0.008;
//             int stop_delay = 200;
//             bool is_deg = false;
//         };

//         RTB_SKILL_PORT(MultiAxisFollowSkill, Motion, fw::MotionBase);
//     };

//     explicit MultiAxisFollowSkill(std::string name);
//     ~MultiAxisFollowSkill() override;
//     MultiAxisFollowSkill(const MultiAxisFollowSkill& other);

//     auto prepare(const Port::Config& cfg) -> void;
//     auto step() -> fw::Status override;
//     auto stop() -> void override;

// private:
//     Port::Config config_;
//     struct Imp;
//     std::unique_ptr<Imp> imp_;
// };

// // 注册 Tag
// TAG_CREATE_AND_LINK(MultiAxisFollowSkill, MultiAxisFollowSkill);

// }  // namespace rtb::skill

// #endif  // SKILL_FOLLOW_HPP