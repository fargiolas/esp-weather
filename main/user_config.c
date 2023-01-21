/*
 * Copyright (c) 2023 Filippo Argiolas <filippo.argiolas@gmail.com>.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <sys/errno.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_libc.h"
#include "esp_err.h"
#include "nvs.h"
#include "sdkconfig.h"
#include "user_config.h"
#include "util.h"

#define TAG "config-parser"

typedef struct _KeyType KeyType;

typedef esp_err_t (*config_read_func) (KeyType key, nvs_handle_t handle, void *data);
typedef esp_err_t (*config_write_func) (KeyType key, nvs_handle_t handle, void *data);

struct _KeyType {
    char *key;
    char *default_value;
    config_read_func read;
    config_write_func write;
};

static void check_error(esp_err_t err, char *key, char *value)
{
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Successfully saved %s=%s to nvs", key, value);
    } else {
        ESP_LOGE("Error while saving %s=%s to nvs: %s", key, value, esp_err_to_name(err));
    }
}

static esp_err_t config_read_str(KeyType key, nvs_handle_t nvs_handle, void *data) {
    esp_err_t err;
    char **value = (char **) data;
    size_t required_size;

    ESP_LOGI(TAG, "reading %s from nvs", key.key);

    err = nvs_get_str(nvs_handle, key.key, NULL, &required_size);
    if (err == ESP_OK) {
        *value = os_malloc(required_size * sizeof(char));
        err = nvs_get_str(nvs_handle, key.key, *value, &required_size);
    }

    if (err != ESP_OK) {
        *value = os_malloc((strlen(key.default_value)+1) * sizeof(char));
        strcpy(*value, key.default_value);
        ESP_LOGW(TAG, "Couldn't load %s from NVS, using default value: %s", key.key, *value);
    }

    return err;
}

static esp_err_t config_write_str(KeyType key, nvs_handle_t nvs_handle, void *data) {
    esp_err_t err;
    char *value = (char *) data;

    ESP_LOGI(TAG, "writing %s to nvs", value);

    err = nvs_set_str(nvs_handle, key.key, value);
    check_error(err, key.key, value);

    return err;
}

static esp_err_t config_read_i32(KeyType key, nvs_handle_t nvs_handle, void *data) {
    esp_err_t err;
    int32_t *value = (int32_t *) data;
    *value = strtoll(key.default_value, NULL, 10);

    ESP_LOGI(TAG, "reading %s from nvs", key.key);

    err = nvs_get_i32(nvs_handle, key.key, value);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Couldn't load %s from NVS, using default value", key.key);
    }

    return err;
}

static esp_err_t config_write_i32(KeyType key, nvs_handle_t nvs_handle, void *data) {
    esp_err_t err;
    char *value = (char *) data;
    int32_t ivalue = (int32_t) strtoll(value, NULL, 10);
    ESP_LOGI(TAG, "writing %d to nvs", ivalue);

    err = nvs_set_i32(nvs_handle, key.key, ivalue);
    check_error(err, key.key, value);

    return err;
}

static KeyType allowed_keys[] = {
    {"wifi_ssid",       CONFIG_ESP_WIFI_SSID,       config_read_str, config_write_str},
    {"wifi_pass",       CONFIG_ESP_WIFI_PASSWORD,   config_read_str, config_write_str},
    {"mqtt_broker_url", CONFIG_BROKER_URL,          config_read_str, config_write_str},
    {"humidity_offset", "0",                        config_read_i32, config_write_i32}
};


static esp_err_t write_to_nvs(nvs_handle_t nvs_handle, char *key, char *value)
{
    int valid = 0;
    esp_err_t err = ESP_OK;

    for (int i=0; i < N_ELEMENTS(allowed_keys); i++) {
        if (strcmp(key, allowed_keys[i].key) == 0) {
            KeyType k = allowed_keys[i];
            err = k.write(k, nvs_handle, value);
            valid = 1;
            return err;
        }
    }

    if (!valid) {
        ESP_LOGW(TAG, "received invalid config key: %s", key);
    }

    return err;
}


static int is_valid(char c)
{
    char *valid_chars = "_-=.,@!#:/";

    for (int i=0; i<strlen(valid_chars); i++) {
        if (c == valid_chars[i]) return 1;
    }

    return isalnum(c);
}

static char *chomp(const char *str, uint32_t len)
{
    char *tmp = (char *) os_zalloc(len+1 * sizeof(char));
    char *out;

    char *i, *o;

    /* strip invalid characters and null terminates input string */
    for (i = (char *) str, o = tmp; (i - str) < len; i++) {
        if (is_valid(*i)) {
            *o = *i;
            o++;
        }
    }

    *o = '\0';

    out = (char *) os_malloc(strlen(tmp) * sizeof(char));
    strcpy(out, tmp);

    os_free(tmp);

    return out;
}

static char *get_pair(char *key_pos, char *key, char *value)
{
    uint8_t key_sz, value_sz;
    char *value_pos;
    char *iter;

    value_pos = key_pos;

    for (iter = key_pos; (*iter != ',') && (*iter != '\0'); iter++) {
        if (*iter == '=') {
            key_sz = iter - key_pos;
            value_pos = iter + 1;
            memcpy(key, key_pos, key_sz * sizeof(char));
            key[key_sz] = '\0';
        }
    }
    value_sz = iter - value_pos;
    memcpy(value, value_pos, value_sz * sizeof(char));
    value[value_sz] = '\0';

    return *iter == '\0' ? NULL : iter + 1;
}

esp_err_t config_read(char *key, void *data) {
    nvs_handle_t nvs_handle;
    esp_err_t err = ESP_OK;
    int valid = 0;

    ESP_ERROR_CHECK(nvs_open("config", NVS_READWRITE, &nvs_handle));

    for (int i=0; i < N_ELEMENTS(allowed_keys); i++) {
        KeyType k = allowed_keys[i];
        if (strcmp(key, k.key) == 0) {
            err = k.read(k, nvs_handle, data);
            valid = 1;
            return err;
        }
    }

    if (!valid) {
        ESP_LOGW(TAG, "invalid config key: %s", key);
    }

    nvs_close(nvs_handle);

    return err;
}

void config_parse_payload(char *msg, size_t msg_len)
{
    nvs_handle_t nvs_handle;
    char key[64], value[64];
    char *cleaned_msg = chomp(msg, msg_len);
    char *next = cleaned_msg;

    ESP_ERROR_CHECK(nvs_open("config", NVS_READWRITE, &nvs_handle));

    do {
        memset(key, 0, N_ELEMENTS(key));
        memset(value, 0, N_ELEMENTS(value));

        next = get_pair(next, key, value);

        ESP_LOGI(TAG, "%s => %s", key, value);

        write_to_nvs(nvs_handle, key, value);
    } while (next != NULL);

    ESP_ERROR_CHECK(nvs_commit(nvs_handle));

    nvs_close(nvs_handle);
    os_free(cleaned_msg);
    ESP_LOGI(TAG, "Restarting the device to load the new configuration");
    esp_restart();
}
