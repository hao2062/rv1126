// 下面代码，基于 V4L2 框架，用于注册视频设别，和管理异步子设备

// 头文件
#include <linux/module.h>  
#include <linux/videodev2.h>  
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h> 
#include <media/v4l2-common.h>  
#include <media/v4l2-ioctl.h>  
#include <media/v4l2-async.h> 

//#include <linux/v4l2-async.h>

// 假设你有一个自定义的驱动结构体  

// 自定义数据结构，包含 V4l2 子设备
struct my_video_device {  
    struct video_device* nodedev;   // 视频设备节点（对应 v4l2 框架的 /dev/video0）， 用于和用户态交互
    struct v4l2_device v4l2_dev;    // v4l2 主设备 （对应 v4l2 host），用于管理 V4L2 子设备
};  

// 定义全局变量 vdev，是一个指针，指向后面分配的结构体实例（struct my_video_device）
struct my_video_device *vdev; 

// 子设备绑定 与 完成回调

// 子设备绑定回调函数：在子设备绑定通知器时调用
// 输入：notifier-指向当前触发事件的异步通知器； subdev-表示成功绑定的子设备    
// 输入：asd-表示匹配到的异步子设备描述符
static int my_bound_handler(struct v4l2_async_notifier *notifier,  
                            struct v4l2_subdev *subdev,  
                            struct v4l2_async_subdev *asd)  
{  
    int ret;
    // 注册子设备节点：在主设备 v4l2_dev 中注册当前绑定的子设备节点
    // 子设备节点在用户空间为：/dev/v4l-subdevX，供 user 访问

    // pr_info("Function: my_bound_handler（）");
    // dump_stack();

    ret=v4l2_device_register_subdev_nodes(notifier->v4l2_dev);
    if(ret<0){
        printk(KERN_INFO "cuowu%d",ret);
    }
    printk(KERN_INFO "Subdevice %s bound to notifier\n", subdev->name);  
    return 0;  
}  
  
// 所有子设备绑定，并完成回调
static int my_complete_handler(struct v4l2_async_notifier *notifier)  
{  
    // 这里处理所有子设备都绑定完成的事件  
    printk(KERN_INFO "All subdevices bound to notifier\n");  
    return 0;  
}

// 定义异步通知器操作函数
static const struct v4l2_async_notifier_operations my_async_ops = {  
    // 子设备绑定到通知器时调用
    .bound = my_bound_handler,
    // 当所有子设备完成绑定后调用  
    .complete = my_complete_handler,  
};

// 定义异步子设备和通知器
static struct v4l2_async_subdev my_async_subdev = {  
    .match_type = V4L2_ASYNC_MATCH_DEVNAME, /* 使用设备名称进行匹配 */  
    .match = {  
        .device_name = "spi1.0", /* 设置你的子设备名称 */  
    },  
    /* list 字段由 v4l2-async 核心管理，不需要手动设置 */  
};  

// static struct v4l2_async_subdev *v4l2_subdev_match[]={
//     {&my_async_subdev},
//     {},

// };

// 定义了一个异步通知器 notifier，用于子设备的异步绑定和时间通知
// 下面的 my_async_ops 表示异步通知器操作函数集的指针，下面定义了子设备 绑定 和 完成绑定的回调函数 
static struct v4l2_async_notifier notifier = {  
    .ops = &my_async_ops,
};

// 文件操作接口函数，用于用户态访问设备节点时调用
// 基本上都为空函数，用于模拟设备的读写操作

// 打开设备节点
static int v4l2_open(struct file *filp){

        printk("open success\n");
        return 0;
}

// 读取数据
static ssize_t v4l2_read(struct file *filp, char __user * buf, size_t cnt, loff_t * off){
    int ret=0;
    char data[7]="zsr";
    ret = copy_to_user(buf, data, sizeof(data));
    return 0;
}

// 关闭设备节点
static int v4l2_release (struct file *filp){
    
    printk("release success\n");
    return 0;
}

// 填充 v4l2 的回调函数
const struct v4l2_file_operations your_fops={
    .owner=THIS_MODULE,
    .open=v4l2_open,
    .read=v4l2_read,
    .release=v4l2_release,
};

// V4L2设备的回调函数（如果需要的话）  
// static const struct v4l2_device_ops my_v4l2_ops = {  
//    ... 填充必要的回调函数 ...  
// };  

// 初始化 V4L2 设备 ，设置设备名 并 注册到系统
static int my_v4l2_device_init(struct my_video_device *vdev)  
{  
    int ret;  
    // 初始化v4l2_device 
    char *name="zsr" ;
    vdev->v4l2_dev.notify = NULL; // 如果需要通知机制，设置回调  
    vdev->v4l2_dev.release = NULL; // 设备释放时的回调（如果需要）  
    memcpy(vdev->v4l2_dev.name, name, 3);
    printk("%s\n",vdev->v4l2_dev.name);
    //vdev->v4l2_dev.ops = &my_v4l2_ops; // 指向你的v4l2_device_ops  
  
    // 注册V4L2设备 
    ret = v4l2_device_register(NULL, &vdev->v4l2_dev);  
    if (ret < 0) {  
        printk(KERN_ERR "Failed to register V4L2 device\n");  
        return ret;  
    }  
  
    // 在这里你可以注册V4L2子设备（如果需要）  
    // 例如：v4l2_subdev_register(&vdev->v4l2_dev, ...);  
  
    // 初始化其他资源...  
  
    return 0;  
}

// 释放 v4l2 设备（及相关资源）
static void my_video_device_release(struct video_device *vdev){
    
    kfree(notifier.subdevs);
    //v4l2_device_unregister()
    v4l2_async_notifier_unregister(&notifier);
    v4l2_device_unregister(vdev->v4l2_dev);
     printk("exit\n");
}

// 驱动初始化函数：分配并初始化视频设备和子设备；注册 V4L2 主设备和异步通知器
static int __init my_video_driver_init(void)  
{  
    int ret;  
      
    // 分配设备结构体  
    //size_t ptr_size = sizeof(struct v4l2_async_subdev *)
    char *name="zsr" ;
    
    // 给私有结构体分配内存
    vdev = kzalloc(sizeof(*vdev), GFP_KERNEL);  
    if (!vdev) {  
        printk(KERN_ERR "Failed to allocate video device\n");  
        return -ENOMEM;  
    }  
    // 初始化 V4L2 设备  
    ret = my_v4l2_device_init(vdev);  
    if (ret < 0) {  
        kfree(vdev);  
        return ret;  
    }  

    // printk("%s\n",*(notifier.subdevs)->match.device_name);
    // 异步通知的子设备列表 需要 动态分配
    if (notifier.subdevs == NULL) {  
        // 手动分配 subdevs 列表
        notifier.subdevs = kzalloc(sizeof(struct v4l2_async_subdev*),GFP_KERNEL);  
        if (notifier.subdevs == NULL) {  
            // 处理内存分配失败  
            printk("shibai\n");
        }  
        // num_subdevs 记录当前绑定的子设备数量
        notifier.num_subdevs = 0;   
    } 
    // 把我们定义的子设备和通知器 添加到 子设备列表
    notifier.subdevs[notifier.num_subdevs++] = &my_async_subdev; 

    printk("%s\n",notifier.subdevs[0]->match.device_name);
    printk("%d\n",sizeof(*(vdev->nodedev)));

    // 为 vdev->nodedev 分配内存，确保视频设备节点可以存储
    vdev->nodedev = kzalloc(sizeof(*(vdev->nodedev)), GFP_KERNEL); 

    // memset(&vdev->nodedev, 0, sizeof(vdev->nodedev));  
    
    // 设置设备类型为视频捕获设备  
    vdev->nodedev->vfl_type = VFL_TYPE_GRABBER; 
    // 设置设备的文件操作函数集  
    vdev->nodedev->fops = &your_fops; 
    //vdev->nodedev->ioctl_ops = &your_ioctl_ops; // 设置ioctl操作函数集  
    // 设置释放函数  
    vdev->nodedev->release = my_video_device_release; 
    //vdev->nodedev.lock = &your_lock; // 设置锁，用于同步访问  
    //vdev->nodedev->name = "VEDIO"; // 设置设备名称  
    memcpy(vdev->nodedev->name, name, 3);   // 设置设备名为 zsr
    printk("%s\n",vdev->nodedev->name);
    vdev->nodedev->v4l2_dev=&vdev->v4l2_dev;    // 把 v4l2_dev 赋值给 nodedev->v4l2_dev，表示该 video 节点归属与主设备

    // 注册视频设备节点，即 /dev/videox
    ret = video_register_device(vdev->nodedev, VFL_TYPE_GRABBER, -1);   // VFL_TYPE_GRABBER - 视频捕获类型
    if(ret<0)
        printk("err\n"); 
    if (ret < 0) {    
        printk(KERN_ERR "Failed to register async subdev: %d\n", ret);   
    }  
    
    printk("nnn\n");
    
    // 把异步通知器注册到 V4L2 主设备，开启子设备的异步匹配和绑定的流程
    v4l2_async_notifier_register(&vdev->v4l2_dev, &notifier);
    
    printk(KERN_INFO "My Video Driver loaded successfully\n");  
    return 0;  
}  

//v4l2_device_register_subdev_nodes

// 驱动清理函数（卸载驱动，释放资源—）  
static void __exit my_video_driver_exit(void)  
{  
    // 在这里执行清理操作，如注销V4L2设备、释放资源等  
    // ... 

    // 注销设备节点 /dev/videoX，停止 user 访问设备
    video_unregister_device(vdev->nodedev);
    // 释放分配的 video_device 内存
    kfree(vdev->nodedev);
    // 指针置为空
    vdev->nodedev=NULL;
    // 释放动态分配的 my_video_device 内存
    kfree(vdev);  
    vdev = NULL;
    // 打印消息表示驱动已卸载  
    printk(KERN_INFO "My Video Driver unloaded\n"); 
}  

// 模块出入口
module_init(my_video_driver_init);  
module_exit(my_video_driver_exit);  
MODULE_LICENSE("GPL");