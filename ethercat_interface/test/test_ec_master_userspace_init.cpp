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
  }

  EXPECT_THAT(
    calls,
    ElementsAre(
      "masters_create:0",
      "request_master:2",
      "wait_for_slave:1",
      "release_master"));
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
