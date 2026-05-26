#ifndef MASTER_SLAVE_HPP_
#define MASTER_SLAVE_HPP_

#include <aris.hpp>

namespace master_slave
{
    class MasterSlave : public aris::core::CloneObject<MasterSlave, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~MasterSlave();
        explicit MasterSlave(const std::string &name = "master_slave");
        
    private:
        struct Imp;
        aris::core::ImpPtr<Imp> imp_;
    };
}

#endif // MASTER_SLAVE_HPP_
