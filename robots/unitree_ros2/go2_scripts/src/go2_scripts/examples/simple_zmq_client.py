import zmq

def receiver():
    # 创建上下文
    context = zmq.Context()
    
    # 创建一个PULL类型的套接字（用于接收消息）
    socket = context.socket(zmq.PULL)
    
    # 连接到发送者的地址
    socket.connect("tcp://192.168.31.158:5555") # Your PC's ip
    print("接收者已启动, 连接到tcp://192.168.31.158:5555")
    
    try:
        # 接收5条消息
        for i in range(5):
            # 接收消息
            message = socket.recv_string()
            print(f"已接收: {message}")
    except KeyboardInterrupt:
        print("\n接收者被用户中断")
    finally:
        # 关闭套接字和上下文
        socket.close()
        context.term()

if __name__ == "__main__":
    receiver()