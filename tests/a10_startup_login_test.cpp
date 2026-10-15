#include "aris.hpp"
#include "kaanh/middleware/shell.hpp"
#include "a10_startup_login.hpp"
#include "kaanhbot/authority/authority.hpp"

#include <iostream>
#include <sstream>
#include <stdexcept>

namespace {

struct TestFailure : std::runtime_error { using std::runtime_error::runtime_error; };

void require(bool condition, const char* message)
{
    if (!condition) throw TestFailure(message);
}

} // namespace

int main()
{
    using kaanhbot::user::User;
    const char* stage = "setup";
    try
    {
        auto& cs = aris::server::ControlServer::instance();
        auto* shell = new kaanh::middleware::Shell;
        cs.resetMiddleWare(shell);
        auto* modules = new aris::core::PointerArray<kaanh::module::MiddleModule>;
        modules->add<kaanhbot::authority::Authority>().setName("Authority");
        auto& users = modules->add<kaanhbot::user::UserManagement>();
        users.setName("UserManagement");
        shell->setModulePool(modules);
        // Install only authentication and authority. UserManagement::init()
        // registers robot telemetry, which this hardware-free test does not need.
        kaanh::log::Sqlite3Log::instance().open("startup_login_test.db");
        stage = "startup login";

        std::ostringstream out, err;
        require(a10_auth::auto_login_manufacturer(out, err), "SDK startup login failed");
        require(users.user() == User::kManufacturer, "SDK role did not become Manufacturer");
        require(out.str().find("自动登录成功") != std::string::npos, "missing success message");
        require(err.str().empty(), "unexpected failure message");
        require(out.str().find(a10_auth::manufacturer_password(std::time(nullptr))) == std::string::npos,
                "credential leaked to success output");

        // The vendor rejects login while already logged in; the helper must be idempotent.
        out.str("");
        require(a10_auth::auto_login_manufacturer(out, err), "repeat startup login failed");
        require(out.str().find("自动登录成功") != std::string::npos, "missing repeated success message");

        stage = "invalid credential";
        users.logout();
        bool rejected = false;
        try { users.login(User::kManufacturer, "invalid-test-credential"); }
        catch (...) { rejected = true; }
        require(rejected && users.user() == User::kNone, "SDK accepted invalid credential");

        // A different existing role must not be reported as a Manufacturer login.
        stage = "existing role";
        const auto test_password = kaanh::general::md5("offline-test-only");
        users.resetEnPwd(test_password);
        users.login(User::kEngineer, test_password);
        out.str("");
        err.str("");
        require(!a10_auth::auto_login_manufacturer(out, err), "wrong role reported as success");
        require(users.user() == User::kEngineer, "existing user was changed");
        require(out.str().empty(), "failure printed success output");
        require(err.str().find("自动登录失败") != std::string::npos, "missing failure message");
        users.logout();

        // The SDK can throw while logging after it has already set the role.
        // User-facing status must still agree with the actual authentication state.
        stage = "audit failure after login";
        kaanh::log::Sqlite3Log::instance().close();
        out.str("");
        err.str("");
        require(a10_auth::auto_login_manufacturer(out, err), "audit failure hid successful login");
        require(users.user() == User::kManufacturer, "audit failure changed login role");
        require(out.str().find("自动登录成功") != std::string::npos, "audit failure hid success message");
        require(err.str().find("SDK 后续处理出现异常") != std::string::npos, "missing audit warning");
        // SDK logout() is noexcept, so restore logging before cleanup.
        kaanh::log::Sqlite3Log::instance().open("startup_login_test.db");
        users.logout();
        kaanh::log::Sqlite3Log::instance().close();

        stage = "missing user module";
        auto* empty_modules = new aris::core::PointerArray<kaanh::module::MiddleModule>;
        empty_modules->add<kaanhbot::authority::Authority>().setName("Authority");
        shell->setModulePool(empty_modules);
        out.str("");
        err.str("");
        require(!a10_auth::auto_login_manufacturer(out, err), "missing module reported success");
        require(out.str().empty(), "missing module printed success output");
        require(err.str().find("自动登录失败") != std::string::npos, "missing module failure message");
        std::cout << "PASS: SDK login, verified role, Chinese messages, repeat login and rejection" << std::endl;
        return 0;
    }
    catch (const TestFailure& failure)
    {
        std::cerr << "FAIL: " << failure.what() << std::endl;
        return 1;
    }
    catch (const std::exception& failure)
    {
        // Setup happens before any password is generated or submitted.
        if (std::string(stage) == "setup") std::cerr << failure.what() << std::endl;
        std::cerr << "FAIL: SDK checks at " << stage << std::endl;
        return 1;
    }
    catch (...)
    {
        // Do not echo SDK exceptions, which may include login arguments.
        std::cerr << "FAIL: offline SDK startup login checks at " << stage << std::endl;
        return 1;
    }
}
