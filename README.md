# `ip_tunnel` ROS 2 Package

## Overview

`ip_tunnel` is a ROS 2 package designed to tunnel IP packets over DDS middleware, enabling IP-based communication between distributed systems within a ROS 2 network. This can be particularly useful in environments where DDS is the primary communication layer, but IP connectivity is required between nodes on separate hosts.

This package provides two main nodes that encapsulate IP packets in ROS 2 messages and transport them across a DDS network, effectively creating an IP tunnel over DDS.

## Theory

Tunneling IP over DDS involves encapsulating IP packets into ROS 2 messages, which are then published and subscribed over DDS using the ROS 2 framework. This setup enables IP packet exchange between hosts using DDS as the underlying transport. Key benefits include:

- **Seamless IP Communication**: Extending IP-level communication across a DDS-based network.
- **Reliable and Flexible Transport**: Leveraging ROS 2 and DDS Quality of Service (QoS) settings to control reliability, latency, and data persistence.
- **Minimal Overhead**: Efficiently reusing existing DDS infrastructure for IP tunneling without needing a separate VPN or dedicated IP link.

In `ip_tunnel`, IP packets are captured and encapsulated as ROS 2 messages, sent across the DDS network, and then decapsulated at the receiving host.

## Installation

1. Clone this repository into your ROS 2 workspace:
    ```
    cd ~/ros2_ws/src && git clone https://github.com/nilsjor/ros-ip-tunnel.git ip_tunnel
    ```

2. Build the package:
    ```
    cd ~/ros2_ws && colcon build --packages-select ip_tunnel
    ```

3. Source your workspace:
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

## Nodes

### `ip_tunnel_node`

This is the core node in the `ip_tunnel` package. It handles both publishing and subscribing to IP-encapsulated messages, and it can act as either the "server" or "client" for the IP tunnel, based on topic arguments provided at runtime.

**Parameters**:

- `pub_topic`: The ROS 2 topic to publish encapsulated IP packets.
- `sub_topic`: The ROS 2 topic to subscribe and listen for encapsulated IP packets.

**Example Usage**:

#### Server Node (Uplink)

To start a node instance that sends uplink packets on `ip_uplink` and receives downlink on `ip_downlink`:
```
ros2 run ip_tunnel ip_tunnel_node ip_uplink ip_downlink
```

#### Client Node (Downlink)

To start a complementary instance that sends downlink packets on `ip_downlink` and receives uplink on `ip_uplink`:
```
ros2 run ip_tunnel ip_tunnel_node ip_downlink ip_uplink
```

## Example Workflow

1. **Run a Server Node** on one host:
    ```
    ros2 run ip_tunnel ip_tunnel_node ip_uplink ip_downlink
    ```
    
2. **Run a Client Node** on the other host:
    ```
    ros2 run ip_tunnel ip_tunnel_node ip_downlink ip_uplink
    ```
    

With these two nodes running, IP packets are tunneled across the ROS 2 network using DDS as the transport layer.
