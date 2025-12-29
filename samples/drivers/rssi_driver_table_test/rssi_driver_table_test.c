#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/slab.h> // [추가] kmalloc 사용을 위해 필요

#define DEVICE_NAME "rssi_driver_table_test" 
#define CLASS_NAME "rssi_class"
#define MAX_NETWORKS 30
#define MAX_SSID_LEN 32

struct wifi_info {
    char ssid[MAX_SSID_LEN];
    int rssi;
    bool active;
};

static struct wifi_info rssi_table[MAX_NETWORKS];
static char line_buffer[256];
static int line_ptr = 0;

static int major_num;
static struct class* rssi_class = NULL;
static struct device* rssi_device = NULL;

void parse_and_update(char *line) {
    char ssid_tmp[MAX_SSID_LEN];
    int rssi_tmp = 0;
    char *s_ptr, *r_ptr;
    // [수정 1] 변수 선언을 맨 위로 이동 (C90 호환성)
    char *src;
    int len;
    int i;

    s_ptr = strstr(line, "SSID:");
    r_ptr = strstr(line, "RSSI:");

    if (s_ptr && r_ptr) {
        if (sscanf(r_ptr + 5, "%d", &rssi_tmp) != 1) {
            printk(KERN_INFO "RSSI_DRV: Parsing failed\n");
            return;
        }

        src = s_ptr + 5;
        len = 0;
        while (src[len] != ',' && src[len] != '\0' && len < MAX_SSID_LEN - 1) {
            ssid_tmp[len] = src[len];
            len++;
        }
        ssid_tmp[len] = '\0';

        for (i = 0; i < MAX_NETWORKS; i++) {
            if (rssi_table[i].active && strcmp(rssi_table[i].ssid, ssid_tmp) == 0) {
                rssi_table[i].rssi = rssi_tmp;
                return;
            }
        }
        for (i = 0; i < MAX_NETWORKS; i++) {
            if (!rssi_table[i].active) {
                strncpy(rssi_table[i].ssid, ssid_tmp, MAX_SSID_LEN - 1);
                rssi_table[i].rssi = rssi_tmp;
                rssi_table[i].active = true;
                return;
            }
        }
    }
}

static ssize_t dev_write(struct file *file, const char __user *buf, size_t len, loff_t *ppos) {
    char k_buf[256];
    int i;

    if (len > sizeof(k_buf) - 1) len = sizeof(k_buf) - 1;
    if (copy_from_user(k_buf, buf, len)) return -EFAULT;
    
    k_buf[len] = '\0'; 

    for (i = 0; i < len; i++) {
        char c = k_buf[i];
        if (c == '\n' || c == '\r') {
            line_buffer[line_ptr] = '\0';
            if (line_ptr > 0) parse_and_update(line_buffer);
            line_ptr = 0;
        } else if (line_ptr < sizeof(line_buffer) - 1) {
            line_buffer[line_ptr++] = c;
        }
    }
    return len;
}

static ssize_t dev_read(struct file *file, char __user *buf, size_t len, loff_t *ppos) {
    // [수정 2] 스택 메모리 대신 kmalloc(동적 할당) 사용
    char *response;
    int pos = 0;
    int i;
    int ret_len = 0;

    if (*ppos > 0) return 0;

    // 커널 메모리 할당 (GFP_KERNEL은 커널 작업용 플래그)
    response = kmalloc(2048, GFP_KERNEL);
    if (!response) return -ENOMEM; // 메모리 부족 시 에러

    pos += sprintf(response + pos, "\n[ WiFi RSSI Monitor Table ]\n");
    pos += sprintf(response + pos, "%-20s | %s\n", "SSID", "RSSI");
    pos += sprintf(response + pos, "--------------------------------\n");

    for (i = 0; i < MAX_NETWORKS; i++) {
        if (rssi_table[i].active) {
            pos += sprintf(response + pos, "%-20s | %d dBm\n", rssi_table[i].ssid, rssi_table[i].rssi);
        }
    }

    if (copy_to_user(buf, response, pos)) {
        kfree(response); // 에러 나면 메모리 해제
        return -EFAULT;
    }

    *ppos = pos;
    ret_len = pos;

    // [중요] 사용한 메모리는 반드시 해제해야 함
    kfree(response);
    
    return ret_len;
}

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .read = dev_read,
    .write = dev_write,
};

static int __init rssi_init(void) {
    major_num = register_chrdev(0, DEVICE_NAME, &fops);
    if (major_num < 0) return major_num;

    rssi_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(rssi_class)) {
        unregister_chrdev(major_num, DEVICE_NAME);
        return PTR_ERR(rssi_class);
    }

    rssi_device = device_create(rssi_class, NULL, MKDEV(major_num, 0), NULL, DEVICE_NAME);
    if (IS_ERR(rssi_device)) {
        class_destroy(rssi_class);
        unregister_chrdev(major_num, DEVICE_NAME);
        return PTR_ERR(rssi_device);
    }

    memset(rssi_table, 0, sizeof(rssi_table));
    printk(KERN_INFO "RSSI Driver: Loaded correctly.\n");
    return 0;
}

static void __exit rssi_exit(void) {
    device_destroy(rssi_class, MKDEV(major_num, 0));
    class_unregister(rssi_class);
    class_destroy(rssi_class);
    unregister_chrdev(major_num, DEVICE_NAME);
    printk(KERN_INFO "RSSI Driver: Unloaded\n");
}

module_init(rssi_init);
module_exit(rssi_exit);
MODULE_LICENSE("GPL");