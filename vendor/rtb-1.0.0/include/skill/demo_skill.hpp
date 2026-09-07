// #include "fw/motion.hpp"
// #include "fw/skill.hpp"

// namespace rtb::skill {

// class MoveSkill : public fw::SkillBase<MoveSkill> {
// public:
//     struct Port {
//         struct Config {
//             double pos[6];
//         };

//         RTB_SKILL_PORT(MoveSkill, Motion, fw::MotionBase);
//     };

//     explicit MoveSkill(std::string name);
//     ~MoveSkill() override;
//     MoveSkill(const MoveSkill& other);

//     auto prepare(const Port::Config& cfg) -> void;
//     auto step() -> fw::Status override;
//     auto stop() -> void override;

// private:
//     struct MoveSkillImp;
//     std::unique_ptr<MoveSkillImp> imp_;
// };

// TAG_CREATE_AND_LINK(MoveSkill, MoveSkill);

// }  // namespace rtb::skill
