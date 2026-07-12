#include "gtest/gtest.h"
#include "estop_bridge/estop_bridge_node.hpp"

// is_estop_command()はKafka I/Oを一切使わない純粋関数として切り出してあるため、
// 実際のKafkaブローカーが無くてもロジックをテストできる
// (Kafka自体との実疎通はエンドツーエンドの手動検証で確認する。demos/07_real_kafka_estop.sh参照)。

TEST(EstopBridgeIsEstopCommand, ExactMatchIsEstop)
{
  EXPECT_TRUE(estop_bridge::EstopBridgeNode::is_estop_command("ESTOP"));
}

TEST(EstopBridgeIsEstopCommand, TrimsWhitespace)
{
  EXPECT_TRUE(estop_bridge::EstopBridgeNode::is_estop_command("  ESTOP\n"));
  EXPECT_TRUE(estop_bridge::EstopBridgeNode::is_estop_command("\tESTOP  "));
}

TEST(EstopBridgeIsEstopCommand, CaseSensitiveAndUnrelatedPayloadsAreIgnored)
{
  EXPECT_FALSE(estop_bridge::EstopBridgeNode::is_estop_command("estop"));
  EXPECT_FALSE(estop_bridge::EstopBridgeNode::is_estop_command("resume"));
  EXPECT_FALSE(estop_bridge::EstopBridgeNode::is_estop_command(""));
  EXPECT_FALSE(estop_bridge::EstopBridgeNode::is_estop_command("ESTOP_CANCEL"));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
