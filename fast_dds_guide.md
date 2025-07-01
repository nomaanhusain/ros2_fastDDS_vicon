# Configuring Fast DDS for communication with ROS 2 topics over multiple machines

## Introduction
This guide explains how to configure Fast DDS Discovery Server to allow ROS topics to be visible across machines in a local network. Useful in scenarios where multiple devices publish and subscribe to one/multiple topics across a network.

To make ROS 2 topics visible across machines:
<ul>
<li>A central Discovery Server must be launched on one machine. This is the FastDDS-server machine</li>
<li>Each participating device must point to this server using the <em>ROS_DISCOVERY_SERVER</em> environment variable.</li>
</ul>
A setup script can simplify the configuration for each machine.

## Setup Instructions
### Step 1: Create the Setup Scripts

Let's create the script ```setup-ros2-discovery.sh```. You’ll create two variants of the this script, differing only in the value of  ```ROS_DISCOVERY_SERVER```. The content of the script is like:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=<DISCOVERY_SERVER_IP>
export ROS_SUPER_CLIENT=True
```

- On the **FastDDS‐server machine** (the host you want to run the discovery server on):
    - Use `127.0.0.1:11811` (localhost) for `<DISCOVERY_SERVER_IP>`.
    - Keep this copy on the server machine only.
    - Get the IP of this machine for the next steps. You could use `ifconfig`, `ipconfig` or `ip a` to find it.

- On **every other (client) machine** that will join the network:
    - Replace `<DISCOVERY_SERVER_IP>` with the FastDDS‐server machine's IP (e.g. 192.168.1.42).
    - Copy this client-variant of the script to each machine that needs to discover/create ROS topics.

Note: If any topics are created by the server machine, they will be also visible across the network.

As an **example**, suppose the IP of the FastDDS-server machine is `192.168.1.42`, then:
- On the server machine the script looks like:
    ```bash
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_DISCOVERY_SERVER=127.0.0.1:11811
    export ROS_SUPER_CLIENT=True
    ```
- On client machine the script looks like:
    ```bash
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_DISCOVERY_SERVER=192.168.1.42:11811
    export ROS_SUPER_CLIENT=True
    ```

### Step 2: Start the Fast DDS Discovery Server

On the **server machine**, open a new terminal and run:
```bash
source setup-ros2-discovery.sh # loads the server variant setup script
fastdds discovery --server-id 0 # starts the discovery service
```

**Note**: Any time you open a new terminal on this machine, re-run the source command before running any ROS 2 nodes.

### Step 3: Source the Setup Script on Clients

On each **client machine**, in every terminal you use for ROS:
```bash
source setup-ros2-discovery.sh # client variant (points to server IP)
```

### Step 4: Launch ROS 2 Nodes

With the server running and all terminals sourced, you can run your ROS 2 nodes or launch files (e.g., your publishers and subscribers).

Now:
- You should be able to create, publish and subscribe to topics across the network.
- `ros2 topic list` should show all the topics on the network.

## Troubleshooting

1. If topics are not visible, try restarting the ROS 2 daemon:
    ```bash
    ros2 daemon stop
    ```
    Then try listing topics again.

2. Make sure the firewall is not blocking port <em>11811</em> on the server machine, if so, use another port.

3. Ensure the setup script is sourced in all terminals before using ROS 2 commands.

<!-- ## Custom Interface

If using custom message types (e.g., from a Vicon system), ensure the corresponding ROS 2 packages are built and sourced on all machines. For example:

```bash
colcon build
source install/setup.bash
```

Failing to source custom interfaces may lead to type errors. -->

## Conclusion
This setup allows ROS 2 nodes across different machines to communicate through a central Fast DDS Discovery Server.

