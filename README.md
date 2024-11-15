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
    sudo ip tuntap add dev tun0 mode tun && \
    sudo ip addr add 10.0.0.1/24 dev tun0 && \
    sudo ip link set dev tun0 up
    ```

2. **Host 2**:
    ```
    sudo ip tuntap add dev tun0 mode tun && \
    sudo ip addr add 10.0.0.2/24 dev tun0 && \
    sudo ip link set dev tun0 up
    ```

This configuration assigns each host a unique IP address on the `10.0.0.0/24` subnet. The on-link route is automatically added, so additional routing configuration is not necessary.

Optionally, you may apply firewall rules to the `tun0` interface on each host. For example, to drop all packets sent by the Husarnet daemon (port 5582):
```
sudo iptables -A OUTPUT -o tun0 -p udp --sport 5582 -j DROP
```

## Node: `ip_tunnel_node`

The `ip_tunnel_node` is the core of the `ip_tunnel` package. It enables IP packet encapsulation and transport over DDS middleware by acting as either the "uplink" or "downlink" endpoint of the tunnel. The node is highly configurable, allowing users to adjust ROS 2 topics and DDS Quality of Service (QoS) settings to suit their application needs.

### Usage

```
ros2 run ip_tunnel ip_tunnel_node --ros-args \
   -p pub_topic:=<pub_topic> \
   -p sub_topic:=<sub_topic> \
   [-p tun_device:=<tun_device>] \
   [-p reliability:=<reliability>] \
   [-p durability:=<durability>] \
   [-p history_depth:=<history_depth>]
```

### Parameters

#### Required Parameters

- **`pub_topic`**: The ROS 2 topic to publish encapsulated IP packets.
- **`sub_topic`**: The ROS 2 topic to subscribe and listen for encapsulated IP packets.

#### Optional Parameters

- **`tun_device`**: Name of the TUN interface. Default: `tun0`.
- **`reliability`**: Reliability policy for DDS QoS. Options:
    - `reliable` _(default)_: Ensures packet delivery but may introduce latency.
    - `best_effort`: Favors low latency but may drop packets.
- **`durability`**: Durability policy for DDS QoS. Options:
    - `volatile` _(default)_: Only keeps data for active subscribers.
    - `transient_local`: Retains data for new subscribers.
- **`history_depth`**: Number of messages to store in the DDS queue. Default: `10`.

For more details about QoS settings, refer to the [ROS 2 QoS Documentation](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html).

## Example Workflow

### Example 1: Default QoS

1. **Run a Server Node** on one host, that sends uplink packets on `ip_uplink` and receives downlink on `ip_downlink`:
    ```
    ros2 run ip_tunnel ip_tunnel_node --ros-args -p pub_topic:=ip_uplink -p sub_topic:=ip_downlink
    ```
    
2. **Run a Client Node** on the other host, that sends downlink packets on `ip_downlink` and receives uplink on `ip_uplink`:
    ```
    ros2 run ip_tunnel ip_tunnel_node --ros-args -p pub_topic:=ip_downlink -p sub_topic:=ip_uplink
    ```

### Example 2: Custom QoS

To use custom QoS settings, specify the parameters:

1. **Server Node**:
    ```
    ros2 run ip_tunnel ip_tunnel_node --ros-args \
       -p pub_topic:=ip_uplink \
       -p sub_topic:=ip_downlink \
       -p reliability:=best_effort \
       -p durability:=volatile \
       -p history_depth:=5
    ```
    
2. **Client Node**:
    ```
    ros2 run ip_tunnel ip_tunnel_node --ros-args \
       -p pub_topic:=ip_downlink \
       -p sub_topic:=ip_uplink \
       -p reliability:=best_effort \
       -p durability:=volatile \
       -p history_depth:=5
    ```

