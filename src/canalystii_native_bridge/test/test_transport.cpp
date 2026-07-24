#include "canalystii_native_bridge/transport.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

namespace cb = canalystii_native_bridge;

namespace
{

cb::CanFrame frame(std::uint32_t id, std::uint8_t marker = 0)
{
  cb::CanFrame result;
  result.id = id;
  result.dlc = 1;
  result.data[0] = marker;
  return result;
}

cb::CanFrame motor_control(std::uint8_t opcode)
{
  cb::CanFrame result;
  result.id = 1;
  result.dlc = 8;
  result.data.fill(0xFF);
  result.data[7] = opcode;
  return result;
}

}  // namespace

TEST(FairTxQueue, AlternatesChannelsWithoutReorderingEachChannel)
{
  cb::FairTxQueue queue({0, 1}, 10);
  ASSERT_TRUE(queue.push(0, frame(1, 10)));
  ASSERT_TRUE(queue.push(0, frame(1, 11)));
  ASSERT_TRUE(queue.push(1, frame(1, 20)));
  ASSERT_TRUE(queue.push(1, frame(1, 21)));

  cb::TxBatch first;
  cb::TxBatch second;
  ASSERT_TRUE(queue.pop(first));
  ASSERT_TRUE(queue.pop(second));
  EXPECT_EQ(first.channel, 0);
  EXPECT_EQ(first.count, 2U);
  EXPECT_EQ(first.frames[0].data[0], 10U);
  EXPECT_EQ(first.frames[1].data[0], 11U);
  EXPECT_EQ(second.channel, 1);
  EXPECT_EQ(second.count, 2U);
  EXPECT_EQ(second.frames[0].data[0], 20U);
  EXPECT_TRUE(queue.empty());
}

TEST(FairTxQueue, PrioritizesControlAcrossChannelsAndPreservesFifo)
{
  cb::FairTxQueue queue({0, 1}, 10);
  ASSERT_TRUE(queue.push(0, frame(1, 10)));
  ASSERT_TRUE(queue.push(1, frame(0x7FF, 20)));
  ASSERT_TRUE(queue.push(0, motor_control(0xFC)));

  cb::TxBatch batch;
  ASSERT_TRUE(queue.pop(batch));
  EXPECT_EQ(batch.channel, 1);
  EXPECT_EQ(batch.frames[0].id, 0x7FFU);

  ASSERT_TRUE(queue.pop(batch));
  EXPECT_EQ(batch.channel, 0);
  EXPECT_EQ(batch.frames[0].data[0], 10U);
  ASSERT_TRUE(queue.pop(batch));
  EXPECT_EQ(batch.channel, 0);
  EXPECT_EQ(batch.frames[0].data[7], 0xFCU);
}

TEST(FairTxQueue, RejectsOverflowAndUnknownChannels)
{
  cb::FairTxQueue queue({0, 1}, 2);
  EXPECT_TRUE(queue.push(0, frame(1)));
  EXPECT_TRUE(queue.push(1, frame(2)));
  EXPECT_FALSE(queue.push(0, frame(3)));
  EXPECT_THROW(queue.push(2, frame(4)), std::invalid_argument);
  EXPECT_EQ(queue.size(), 2U);
}

TEST(FairTxQueue, RecognizesOnlySafetyControlFrames)
{
  EXPECT_TRUE(cb::is_control_transaction(frame(0x7FF)));
  EXPECT_TRUE(cb::is_control_transaction(motor_control(0xFC)));
  EXPECT_TRUE(cb::is_control_transaction(motor_control(0xFD)));
  EXPECT_TRUE(cb::is_control_transaction(motor_control(0xFE)));
  EXPECT_FALSE(cb::is_control_transaction(motor_control(0xFB)));
  EXPECT_FALSE(cb::is_control_transaction(frame(1)));
}
