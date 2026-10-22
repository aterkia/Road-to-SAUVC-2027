// Loopback-only FCU fixture using upstream generated MAVLink, not a vehicle driver.
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <poll.h>
#include <csignal>
#include <chrono>
#include <iostream>
#include <string>
#include <mavlink/v2.0/common/mavlink.h>

volatile std::sig_atomic_t running = 1;

int main(int argc, char **argv) {
  const bool arming_enabled = argc == 2 && std::string(argv[1]) == "--arming";
  bool armed = false;
  std::signal(SIGINT, [](int) {
    running = 0;
  });
  const int fd = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in local{};
  local.sin_family = AF_INET;
  local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (fd < 0 || bind(fd, reinterpret_cast<sockaddr *>(&local), sizeof(local)) < 0)
    return 1;
  socklen_t length = sizeof(local);
  if (getsockname(fd, reinterpret_cast<sockaddr *>(&local), &length) < 0)
    return 1;
  std::cout << ntohs(local.sin_port) << std::endl;
  const auto started = std::chrono::steady_clock::now();
  sockaddr_in remote{};
  bool have_remote = false;
  mavlink_message_t incoming{};
  mavlink_status_t parser{};
  auto send = [&](const mavlink_message_t &msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    auto size = mavlink_msg_to_send_buffer(buffer, &msg);
    sendto(fd, buffer, size, 0, reinterpret_cast<sockaddr *>(&remote), sizeof(remote));
  };
  while (running) {
    pollfd input{fd, POLLIN, 0};
    poll(&input, 1, 50);
    if (input.revents & POLLIN) {
      uint8_t buffer[2048];
      length = sizeof(remote);
      auto count =
          recvfrom(fd, buffer, sizeof(buffer), 0, reinterpret_cast<sockaddr *>(&remote), &length);
      have_remote = count > 0;
      for (ssize_t i = 0; i < count; ++i)
        if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &incoming, &parser)) {
          std::cout << "RX " << incoming.msgid;
          if (incoming.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
            mavlink_command_long_t command{};
            mavlink_msg_command_long_decode(&incoming, &command);
            std::cout << " COMMAND " << command.command;
            if (command.command == MAV_CMD_COMPONENT_ARM_DISARM)
              std::cout << " VALUE " << command.param1 << " FORCE " << command.param2;
            if (arming_enabled) {
              uint8_t result = MAV_RESULT_UNSUPPORTED;
              if (command.command == MAV_CMD_COMPONENT_ARM_DISARM &&
                  (command.param1 == 0 || command.param1 == 1) && command.param2 == 0) {
                armed = command.param1 == 1;
                result = MAV_RESULT_ACCEPTED;
              } else if (command.command == MAV_CMD_REQUEST_MESSAGE ||
                  command.command == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES) {
                result = MAV_RESULT_ACCEPTED;
              }
              mavlink_message_t ack{};
              mavlink_msg_command_ack_pack(1, 1, &ack, command.command, result, 0, 0,
                  incoming.sysid, incoming.compid);
              send(ack);
            }
          }
          std::cout << std::endl;
        }
    }
    if (!have_remote)
      continue;
    const auto ms = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                              std::chrono::steady_clock::now() - started)
                                              .count() +
        1000);
    mavlink_message_t msg{};
    mavlink_msg_heartbeat_pack(1, 1, &msg, MAV_TYPE_SUBMARINE, MAV_AUTOPILOT_ARDUPILOTMEGA,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0),
        0, armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY);
    send(msg);
    // NED north yaw=0 should emerge from MAVROS as ENU yaw=+pi/2.
    mavlink_msg_attitude_pack(1, 1, &msg, ms, 0, 0, 0, 0, 0, 0);
    send(msg);
    // This pressure is a fixture value, not an assumed Pixhawk depth sensor.
    mavlink_msg_scaled_pressure_pack(1, 1, &msg, ms, 1196.133f, 0, 2000, 0);
    send(msg);
  }
  close(fd);
}
