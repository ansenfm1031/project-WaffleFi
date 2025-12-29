#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/string.h>

#define DEVICE_NAME "rssi_dev"
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

void parse_and_update(char *line) {
    char ssid_tmp[MAX_SSID_LEN];
    int rssi_tmp = 0;
    char *s_ptr, *r_ptr;
    int i;

    // 1. "SSID:"와 "RSSI:" 키워드 위치 확인
    s_ptr = strstr(line, "SSID:");
    r_ptr = strstr(line, "RSSI:");

    if (s_ptr && r_ptr) {
        // 2. RSSI 값 추출 (단순하게 r_ptr + 5 위치에서 정수 읽기)
        if (sscanf(r_ptr + 5, "%d", &rssi_tmp) != 1) {
            printk(KERN_INFO "RSSI_DRV: RSSI value parsing failed\n");
            return;
        }

        // 3. SSID 값 추출 ("SSID:" 뒤부터 "," 전까지)
        char *src = s_ptr + 5;
        int len = 0;
        while (src[len] != ',' && src[len] != '\0' && len < MAX_SSID_LEN - 1) {
            ssid_tmp[len] = src[len];
            len++;
        }
        ssid_tmp[len] = '\0';

        // [디버깅] 이제 성공 로그가 찍힐 겁니다.
        printk(KERN_INFO "RSSI_DRV: SUCCESS! SSID:[%s] RSSI:[%d]\n", ssid_tmp, rssi_tmp);

        // 4. 테이블 업데이트 (기존과 동일)
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
    } else {
        printk(KERN_INFO "RSSI_DRV: Keyword match failed -> [%s]\n", line);
    }
}

static ssize_t dev_write(struct file *file, const char __user *buf, size_t len, loff_t *ppos) {
    char k_buf[256];
    int i;

    if (len > sizeof(k_buf)) len = sizeof(k_buf);
    if (copy_from_user(k_buf, buf, len)) return -EFAULT;

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
    // echo 명령어는 끝에 \n을 붙이지만, 혹시 안 붙어올 경우를 대비해 
    // line_ptr가 0이 아니면 여기서 한 번 더 처리할 수도 있으나 우선은 유지합니다.
    return len;
}

static ssize_t dev_read(struct file *file, char __user *buf, size_t len, loff_t *ppos) {
    char response[2048];
    int pos = 0;
    int i;

    if (*ppos > 0) return 0;

    pos += sprintf(response + pos, "\n[ WiFi RSSI Monitor Table ]\n");
    pos += sprintf(response + pos, "%-20s | %s\n", "SSID", "RSSI");
    pos += sprintf(response + pos, "--------------------------------\n");

    for (i = 0; i < MAX_NETWORKS; i++) {
        if (rssi_table[i].active) {
            pos += sprintf(response + pos, "%-20s | %d dBm\n", rssi_table[i].ssid, rssi_table[i].rssi);
        }
    }

    if (copy_to_user(buf, response, pos)) return -EFAULT;
    *ppos = pos;
    return pos;
}

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .read = dev_read,
    .write = dev_write,
};

static int __init rssi_init(void) {
    major_num = register_chrdev(0, DEVICE_NAME, &fops);
    memset(rssi_table, 0, sizeof(rssi_table));
    printk(KERN_INFO "RSSI_DRV: Standby... Use echo to test.\n");
    return 0;
}

static void __exit rssi_exit(void) {
    unregister_chrdev(major_num, DEVICE_NAME);
}

module_init(rssi_init);
module_exit(rssi_exit);
MODULE_LICENSE("GPL");