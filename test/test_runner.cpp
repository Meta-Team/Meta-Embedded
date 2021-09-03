//
// Created by liuzikai on 8/17/21.
//

#include <CppUTest/CommandLineTestRunner.h>
#include <CppUTest/TestRegistry.h>
#include <CppUTestExt/MockSupportPlugin.h>

MockSupportPlugin mockPlugin;

int main(int ac, char** av)
{
    SetPointerPlugin ps("PointerStore");
    TestRegistry::getCurrentRegistry()->installPlugin(&ps);

    TestRegistry::getCurrentRegistry()->installPlugin(&mockPlugin);

    return CommandLineTestRunner::RunAllTests(ac, av);
}