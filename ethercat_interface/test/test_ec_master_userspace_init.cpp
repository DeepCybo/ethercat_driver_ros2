// Copyright 2026 ICUBE Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gmock/gmock.h>

#include <string>
#include <vector>

#include "ethercat_interface/ec_master.hpp"

namespace
{

using ::testing::ElementsAre;

TEST(TestEcMasterDcClock, UsesMonotonicApplicationClock)
{
  EXPECT_EQ(CLOCK_MONOTONIC, ethercat_interface::EcMaster::applicationClockId());
}

TEST(TestEcMasterDcClock, ConvertsApplicationTimeWithoutEpochOffset)
{
  const timespec time = {1, 234};
  EXPECT_EQ(1000000234ULL, ethercat_interface::EC_TIMESPEC2NANO(time));
}

TEST(TestEcMasterUserspaceInit, CreatesUserspaceMasterBeforeRequestingMaster)
{
  std::vector<std::string> calls;
  auto fake_master = reinterpret_cast<ec_master_t *>(0x1);
  bool scan_busy = false;
  uint8_t slave_state = EC_AL_STATE_PREOP;

  ethercat_interface::EcMasterEcrtApi api;
  api.masters_create = [&](unsigned int node_id) {
      calls.push_back("masters_create:" + std::to_string(node_id));
      return fake_master;
    };
  api.request_master = [&](unsigned int master_id) {
      calls.push_back("request_master:" + std::to_string(master_id));
      return fake_master;
    };
  api.master_wait_for_slave = [&](ec_master_t * master, int slave_count) {
      EXPECT_EQ(master, fake_master);
      calls.push_back("wait_for_slave:" + std::to_string(slave_count));
      return 0;
    };
  api.master_info = [&](ec_master_t * master, ec_master_info_t * info) {
      EXPECT_EQ(master, fake_master);
      calls.push_back("master_info");
      info->link_up = 1;
      info->scan_busy = scan_busy;
      info->slave_count = 1;
      return 0;
    };
  api.master_get_slave = [&](
    ec_master_t * master, uint16_t position, ec_slave_info_t * info) {
      EXPECT_EQ(master, fake_master);
      EXPECT_EQ(position, 0);
      calls.push_back("master_get_slave:0");
      info->al_state = slave_state;
      return 0;
    };
  api.release_master = [&](ec_master_t * master) {
      EXPECT_EQ(master, fake_master);
      calls.push_back("release_master");
    };

  ethercat_interface::EcMasterOptions options;
  options.master_id = 2;
  options.userspace_node_id = 0;
  options.create_userspace_master = true;
  options.wait_for_slave_count = 1;

  {
    ethercat_interface::EcMaster master(options, api);
    EXPECT_THAT(calls, ElementsAre("masters_create:0", "request_master:2"));
    calls.clear();
    EXPECT_TRUE(master.configuredSlavesReady());
    EXPECT_THAT(calls, ElementsAre("master_info", "master_get_slave:0"));

    calls.clear();
    scan_busy = true;
    EXPECT_FALSE(master.configuredSlavesReady());
    EXPECT_THAT(calls, ElementsAre("master_info"));

    calls.clear();
    scan_busy = false;
    slave_state = EC_AL_STATE_INIT;
    EXPECT_FALSE(master.configuredSlavesReady());
    EXPECT_THAT(calls, ElementsAre("master_info", "master_get_slave:0"));
    calls.clear();
  }

  EXPECT_THAT(calls, ElementsAre("release_master"));
}

TEST(TestEcMasterUserspaceInit, SkipsUserspaceCreateAndWaitWhenDisabled)
{
  std::vector<std::string> calls;
  auto fake_master = reinterpret_cast<ec_master_t *>(0x1);

  ethercat_interface::EcMasterEcrtApi api;
  api.masters_create = [&](unsigned int) {
      calls.push_back("unexpected_masters_create");
      return fake_master;
    };
  api.request_master = [&](unsigned int master_id) {
      calls.push_back("request_master:" + std::to_string(master_id));
      return fake_master;
    };
  api.master_wait_for_slave = [&](ec_master_t *, int) {
      calls.push_back("unexpected_wait_for_slave");
      return 0;
    };
  api.release_master = [&](ec_master_t * master) {
      EXPECT_EQ(master, fake_master);
      calls.push_back("release_master");
    };

  ethercat_interface::EcMasterOptions options;
  options.master_id = 0;
  options.create_userspace_master = false;
  options.wait_for_slave_count = 0;

  {
    ethercat_interface::EcMaster master(options, api);
  }

  EXPECT_THAT(calls, ElementsAre("request_master:0", "release_master"));
}

}  // namespace
