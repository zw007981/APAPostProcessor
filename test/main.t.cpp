#include <gtest/gtest.h>

#include "util/logger.h"

int main(int argc, char** argv) {
    apa_post_processor::Logger::SetLogDirectory("../log");
    apa_post_processor::Logger::SetConsoleOutputEnabled(false);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}