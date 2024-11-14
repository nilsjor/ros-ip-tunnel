# `ip_tunnel` ROS 2 Package

## Overview

`ip_tunnel` is a ROS 2 package that enables IP packet tunneling over DDS middleware, allowing users to leverage familiar IP-based tools (e.g., `ping`, `iperf`) to test and benchmark DDS Quality of Service (QoS) settings. By encapsulating IP packets as ROS 2 messages, `ip_tunnel` allows seamless IP communication between distributed systems without the need for VPNs or dedicated networking hardware.

This package facilitates IP packet exchange across a DDS-based network by encapsulating IP packets in ROS 2 messages, which are then published and subscribed over DDS. This setup provides a flexible testing environment where users can evaluate DDS QoS profiles, monitor latency, reliability, and data persistence, and validate network behavior under various QoS configurations.

### Key Benefits

- **Flexible IP Communication Over DDS**: Extend IP-level communication across a DDS-based network, enabling easy IP-based testing and benchmarking.
- **Configurable QoS Settings**: Use ROS 2 and DDS QoS profiles to control reliability, latency, and data persistence, tailoring performance to suit specific needs.
- **Efficient Infrastructure Reuse**: Avoid additional tools and configurations by reusing existing DDS infrastructure for IP tunneling, eliminating the need for a VPN or dedicated link.

## Installation

1. Navigate to your ROS 2 workspace:
    ```
    cd ~/ros2_ws/src
    ```

2. Clone this repository into your workspace:
    ```
    git clone https://github.com/nilsjor/ros-ip-tunnel.git ip_tunnel
    ```

3. Build the package:
    ```
    colcon build --packages-select ip_tunnel
    ```

4. Source your workspace:
    ```
    source install/setup.bash
    ```

## Configuring TUN Interfaces

On each host, create and configure a unique TUN interface with an IP address on the same subnet (e.g., `10.0.0.0/24`). This setup allows both hosts to communicate directly through the TUN interfaces.

### Step-by-Step Setup

1. **Host 1**:
    ```
    sudo ip tuntap add dev tun0 mode tun; \
    sudo ip addr add 10.0.0.1/24 dev tun0; \
    sudo ip link set dev tun0 up
    ```

2. **Host 2**:
    ```
    sudo ip tuntap add dev tun0 mode tun; \
    sudo ip addr add 10.0.0.2/24 dev tun0; \
    sudo ip link set dev tun0 up
    ```

This configuration assigns each host a unique IP address on the `10.0.0.0/24` subnet. The on-link route is automatically added, so additional routing configuration is not necessary.

Optionally, it may useful to apply firewall rules to the `tun0` interface on each host.
For example, to drop all packets sent the Husarnet daemon (port 5582):
```
sudo iptables -A OUTPUT -o tun0 -p udp --sport 5582 -j DROP
```

## Nodes

### `ip_tunnel_node`

This is the core node in the `ip_tunnel` package. It handles both publishing and subscribing to IP-encapsulated messages, and it can act as either the "uplink" or "downlink" for the IP tunnel, based on topic arguments provided at runtime.

```
ros2 run ip_tunnel ip_tunnel_node --ros-args -p <sub_topic> -p <pub_topic> -p [tun_device]
```

**Parameters**:

- `pub_topic`: The ROS 2 topic to publish encapsulated IP packets.
- `sub_topic`: The ROS 2 topic to subscribe and listen for encapsulated IP packets.
- `tun_device`: The name of the TUN interface. Optional, default: `tun0`.

## Example Workflow

1. **Run a Server Node** on one host, that sends uplink packets on `ip_uplink` and receives downlink on `ip_downlink`:
    ```
    ros2 run ip_tunnel ip_tunnel_node --ros-args -p pub_topic:=ip_uplink -p sub_topic:=ip_downlink
    ```
    
2. **Run a Client Node** on the other host, that sends downlink packets on `ip_downlink` and receives uplink on `ip_uplink`:
    ```
    ros2 run ip_tunnel ip_tunnel_node --ros-args -p pub_topic:=ip_downlink -p sub_topic:=ip_uplink
    ```
    
With these two nodes running, IP packets are tunneled across the ROS 2 network using DDS as the transport layer.
