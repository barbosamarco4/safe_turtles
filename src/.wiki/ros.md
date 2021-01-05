* First call: 
    ros::init(argc,argv,<node_name>);
* Create a handle for this node:
    ros::NodeHandle n;
* Subscribing topics:
    ros::Subscriber sub = n.subscribe(<topic>, queue_size, callback);
* Publish to topic:
        _advertise to master the topic I wish to publish_
    ros::Publisher pub = n.advertisei<T>(<topic>i, queue_size);
    pub.publish(msg);
* Spin
    * ros :: spin() -> serves callbacks untill program is terminated
    * ros :: spinOnce() -> serves 1 callback and returns
