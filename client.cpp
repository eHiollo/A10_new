#include "aris.hpp"
#include "kaanh/general/json.hpp"
#include <ctime>
#include <cstdlib>


auto fromJsonFile(const std::filesystem::path &file)->std::string{
    std::ifstream fs(file);

    std::string str((std::istreambuf_iterator<char>(fs)), std::istreambuf_iterator<char>());

    fs.close();
    return str;
}

auto follower_aj()->void
{    
    aris::core::Socket client;
    client.setConnectType(aris::core::Socket::Type::UDP);
    client.setOnReceivedMsg([](aris::core::Socket* s, aris::core::Msg& msg)->int {
        std::cout << "client received:" << msg.toString() << std::endl;
        return 0;
    });
    client.connect("127.0.0.1", "9999");

    {
        std::vector<double> pos;


        int id = 0;
        while(true){
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            nlohmann::json j;

            double random_number = (double)rand()/ RAND_MAX * (90 - (-90)) + (-90);
            // pos.resize(14,random_number*aris::PI/180.0);
            pos.resize(14,90);

            j["motor_pos"] = pos;
            j["motor_vel"] = pos;
            j["id"] = id++;
            aris::core::Msg msg;
            msg.copy(j.dump());
            std::cout<<"msg send: "<<j.dump()<<std::endl;
            client.sendMsg(msg);

        }
    }


}

auto follower_cart()->void
{
    aris::core::Socket client;
    client.setConnectType(aris::core::Socket::Type::UDP);
    client.setOnReceivedMsg([](aris::core::Socket* s, aris::core::Msg& msg)->int {
        std::cout << "client received:" << msg.toString() << std::endl;
        return 0;
    });
    client.connect("127.0.0.1", "9998");

    // test cart
    {
        // std::vector<std::vector<double>> pe {{0.020,0.020,0.020,0,0,0},{0.010,0.010,0.010,0,0,0}};
   
        int id = 0;

        while(true){

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            double random_number = (double)rand()/ RAND_MAX * (20 - (-20)) + (-20);
            std::vector<std::vector<double>> pe {{random_number/1000.0,random_number/1000.0,random_number/1000.0,0,0,0},{0.010,0.010,0.010,0,0,0}};
            // std::vector<std::vector<double>> pe {{random_number/1000.0,0,0,0,0,0},{0,0,0,0,0,0}};
            nlohmann::json j;
            j["pe"] = pe;
            j["id"] = id++;
            aris::core::Msg msg;
            msg.copy(j.dump());
            std::cout<<"msg send: "<<j.dump()<<std::endl;
            client.sendMsg(msg);

        }
    }

    return ;
}
int main(){
   

follower_aj();

//     // use data in file
//     {
//         auto str =  fromJsonFile("./data.json");

//         nlohmann::json j = nlohmann::json::parse(str);
//         std::cout << j.size() << std::endl;

//         auto pos = j.get<std::vector<std::vector<double>>>();
//         for (size_t i = 0; i < pos.size(); i++)
//         {
//             aris::dynamic::dsp(1,pos[i].size(),pos[i].data());
//         }

//         std::vector<double> p,v;
//         int id = 0;
//         double pp=0.0;
// //        p.resize(14,0.0);
// //        v.resize(14,0.0);

//         p.resize(1, 0.0);
//         v.resize(1, 0.0);
//         for (size_t i = 0; i < pos.size(); i++)
//         {
//             std::this_thread::sleep_for(std::chrono::milliseconds(30));

//             nlohmann::json j;

//             j["motor_pos"] = std::vector<double>(1, pos[i][0]);
//             j["motor_vel"] = v;
//             j["id"] = id++;
//             aris::core::Msg msg;
//             msg.copy(j.dump());
//             // std::cout<<"msg send: "<<j.dump()<<std::endl;
//             client.sendMsg(msg);

//         }
//     }

    

    return 0;
}
