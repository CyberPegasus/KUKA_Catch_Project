# -*- coding: UTF-8 -*-
from socket import *
from xml.dom import minidom
# socket配置
host = '192.168.1.5'
host_kuka = '192.168.1.234'
port_s = 8888
port = 54600
buffer_size = 10
buffer_size_c = 1024
# 机械臂状态
joint_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# 抓取高度，以机械臂上位机的位置为准
h = 265.0


def write_xml(filename, cmd, x, y, z, a, b, c):
    print("Writing Command to XML")
    doc = minidom.Document()
    # 主节点Robots
    robots = doc.createElement("Robots")
    doc.appendChild(robots)
    # 命令节点Command，一级节点
    command = doc.createElement("Command")
    # 写入Command的值
    command_value = doc.createTextNode(str(cmd))
    command.appendChild(command_value)
    robots.appendChild(command)
    # 姿态节点Pos，一级节点
    pos = doc.createElement("Pos")
    x_value = pos.setAttribute("X", str(x))
    y_value = pos.setAttribute("Y", str(y))
    z_value = pos.setAttribute("Z", str(z))
    a_value = pos.setAttribute("A", str(a))
    b_value = pos.setAttribute("B", str(b))
    c_value = pos.setAttribute("C", str(c))
    robots.appendChild(pos)
    # 从XML转为Bytes类型
    write_data = doc.toprettyxml(indent="\t", newl="\n", encoding="utf-8")
    # f = open(filename, "wb")
    # f.write()
    # f.close()
    return write_data


def read_xml(recv_message):
    print("reading message from XML")
    doc = minidom.parseString(str(recv_message, encoding='utf8'))
    # doc = minidom.parse("test_recv.xml")
    root = doc.getElementsByTagName("Joint")[0]
    joint_state[0] = float(root.getAttribute("A1"))
    joint_state[1] = float(root.getAttribute("A2"))
    joint_state[2] = float(root.getAttribute("A3"))
    joint_state[3] = float(root.getAttribute("A4"))
    joint_state[4] = float(root.getAttribute("A5"))
    joint_state[5] = float(root.getAttribute("A6"))
    print("The current joints of robotic arm are as follow:")
    print(joint_state)


def socket_client(ctrl_command):
    addr_c = (host_kuka, port)
    ctrl_client = socket(AF_INET, SOCK_STREAM)
    # 连接机械臂服务端
    print('Begin to connect...')
    ctrl_client.connect(addr_c)
    print('Connection Established')
    print(ctrl_command)
    # 发送控制命令
    # send_data = bytes(ctrl_command, encoding="utf8")
    ctrl_client.send(ctrl_command)
    print('Send Successfully')
    recv_data = ctrl_client.recv(buffer_size_c)
    print('Receive Successfully....Begin to parse')
    read_xml(recv_data)
    ctrl_client.close()


def to_catch(world_x, world_y, world_theta):
    xyzabc = generate_xyzabc(world_x, world_y, world_theta, h)
    # 写入XML格式控制命令，cmd=0🥌时结束程序
    ctrl_command = write_xml("EkiCtrl.xml", 1, xyzabc[0], xyzabc[1], xyzabc[2], xyzabc[3], xyzabc[4], xyzabc[5])
    # 发送控制命令并读取回传信息
    socket_client(ctrl_command)


def generate_xyzabc(world_x, world_y, world_theta, height):
    xyzabc = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    xyzabc[0] = world_x
    xyzabc[1] = world_y
    xyzabc[2] = height
    xyzabc[3] = world_theta  # 从-80度到-170度变化
    xyzabc[4] = -2.0
    xyzabc[5] = -172.0
    return xyzabc


def main():
    addr = (host, port_s)
    ctrl_server = socket(AF_INET, SOCK_STREAM)
    print('Begin to connect...')
    ctrl_server.bind(addr)
    ctrl_server.listen(2)
    print('Listening....')
    while True:
        connection_c, address_c = ctrl_server.accept()
        print('Connection Established,the ip:' + str(address_c) + 'is accepted.')
        recv_data_x = connection_c.recv(buffer_size).decode('UTF-8', 'ignore').strip().strip(b'\x00'.decode())
        print('Receive X Successfully:')
        world_x = float(recv_data_x)
        print(world_x)
        # 去除填充buffer的不可见\x00字符
        recv_data_y = connection_c.recv(buffer_size).decode('UTF-8', 'ignore').strip().strip(b'\x00'.decode())
        print('Receive Y Successfully:')
        world_y = float(recv_data_y)
        print(world_y)
        recv_data_theta = connection_c.recv(buffer_size).decode('UTF-8', 'ignore').strip().strip(b'\x00'.decode())
        print('Receive theta Successfully:')
        world_theta = float(recv_data_theta)
        print(world_theta)
        connection_c.send(b'Already Read.')
        connection_c.close()
        to_catch(world_x, world_y, world_theta)
        str_cmd = input("Enter 1 to continue:")
        if str_cmd != '1':
            break
    ctrl_server.close()


if __name__ == "__main__":
    main()
