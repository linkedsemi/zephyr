/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wolfssl/wolfcrypt/settings.h>
#include <wolfssl/ssl.h>
#include <zephyr/kernel.h>
#include <zephyr/posix/arpa/inet.h>

#define LOG_LEVEL LOG_LEVEL_DBG
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

uint8_t server_cert[] = "-----BEGIN CERTIFICATE-----\n"
"MIIDSTCCAjECFH3QXyfwVWnXKo0NBGx4Y+IrVQ50MA0GCSqGSIb3DQEBCwUAMGEx\n"
"CzAJBgNVBAYTAkNOMQswCQYDVQQIDAJDTjELMAkGA1UEBwwCQ04xCzAJBgNVBAoM\n"
"AkNOMQswCQYDVQQLDAJDTjELMAkGA1UEAwwCQ04xETAPBgkqhkiG9w0BCQEWAkNO\n"
"MB4XDTI0MDgxNDA5MzQ1NVoXDTI1MDgxNDA5MzQ1NVowYTELMAkGA1UEBhMCQ04x\n"
"CzAJBgNVBAgMAkNOMQswCQYDVQQHDAJDTjELMAkGA1UECgwCQ04xCzAJBgNVBAsM\n"
"AkNOMQswCQYDVQQDDAJDTjERMA8GCSqGSIb3DQEJARYCQ04wggEiMA0GCSqGSIb3\n"
"DQEBAQUAA4IBDwAwggEKAoIBAQDI2SHfvLPiVkN8OAj+kdkBTNIfwHjYqHVk5rza\n"
"QQy/q9QKRWkXfDiP2k/qPCFGEIaEpm6KTClyVw0IoZo4PdWqLl4S1DBkkfFg5CYE\n"
"4jR0vHIaqm0KwdR1xQRIN2cbm76MCoF9MyCTmQhrp11fjSJyu3MxwnbmKqvknWQ6\n"
"9QDJlGQK8UfBridk3Njeq8WaHQ/Z3+KSrPyFdPBgV0lTvqOPe+deCd/eTtUS4CPP\n"
"MTvQSXL2kK9M5b7TyRFo1KvkuOhSzbYzW8i22mf3IRvmLw4EDzlA46XbOmogrl3I\n"
"lLiG98jpEiW66P8crlxGw3Ednv22M9qNLDFHlccPKZo0J/yDAgMBAAEwDQYJKoZI\n"
"hvcNAQELBQADggEBAC877eQK4rBEUWW1/IK2IiWvvxBZfH5JUAYwY3ncBC07bYPI\n"
"zo7n4UhVfjC9saX+p6njIoxJlLhruKX2eu1RmurszoZeJQgLVgWUpl8VFfAKunpV\n"
"WCDibDpTdWpl7Dxbwar+z2oNj42BZDjj0MWN8TEq9E5JGt0l+iU4dY4j/3gPtN64\n"
"G3F/RvMJfbTO0BYQYwxulpnjvGlwBPWnMxA1ZXpX6n71xmpX5x4UsNe6kZIz8Aye\n"
"8c06cIcez0T+JLTZhhKAyDlxTGnM+IAGbLz96MY06k3tj3Bnef1JtiE5+2qybniS\n"
"s9J2kYWMZf2kmj3jTP8ta1AyqRLZ/TS92/OdCMs=\n"
"-----END CERTIFICATE-----\n";

uint8_t server_key[] = "-----BEGIN PRIVATE KEY-----\n"
"MIIEvQIBADANBgkqhkiG9w0BAQEFAASCBKcwggSjAgEAAoIBAQDI2SHfvLPiVkN8\n"
"OAj+kdkBTNIfwHjYqHVk5rzaQQy/q9QKRWkXfDiP2k/qPCFGEIaEpm6KTClyVw0I\n"
"oZo4PdWqLl4S1DBkkfFg5CYE4jR0vHIaqm0KwdR1xQRIN2cbm76MCoF9MyCTmQhr\n"
"p11fjSJyu3MxwnbmKqvknWQ69QDJlGQK8UfBridk3Njeq8WaHQ/Z3+KSrPyFdPBg\n"
"V0lTvqOPe+deCd/eTtUS4CPPMTvQSXL2kK9M5b7TyRFo1KvkuOhSzbYzW8i22mf3\n"
"IRvmLw4EDzlA46XbOmogrl3IlLiG98jpEiW66P8crlxGw3Ednv22M9qNLDFHlccP\n"
"KZo0J/yDAgMBAAECggEARWBHJJ7sbdKJRgefB7v+PsY30JLtyzzJLBzNgAA3NJ/Y\n"
"QuH97ohQi1QIBatWfPqpVHmetjOfn2i1TlVvzVyCiOGySgO0YZDN1T+JaGdwYiBT\n"
"s6VJvAz+4901YPKOMYmnFH1ug/4ckw8pyvHJPX+lNgdMv5Ph+RglPvckz36gkoAL\n"
"wHwD0PKWIi6uW0z/a0pMR3GIWET8wPmIEU094ii4dW1UzvBgiZUwIoj6EMIAY4vp\n"
"A56pk+us44nyMWnIOz9IkAIXt3b6evI+LhDX/27Ih9njQXfKlAHrx98LhOrji4Sz\n"
"LdtVPYWBJIIWVuWq9iNCO4bO3JP9ARnVB2wK6B5RpQKBgQD0yIa7ZJ300F3SW16r\n"
"/SU9hBaJ3Me7WgPO8qsDaNV1gXKm848wtmdfxFAjAi8VmwqJ5d8kru70bPzLL0GY\n"
"lEFJf0w7nCa3zXono/wWmK+UcWpwqSllVJWZjdebkmZ5+QTgnbqxSh36Q+M4ADrC\n"
"XTPJOcEVWoXz3y3hpjWPbLsURwKBgQDSDTea4O3aK+eHRgszsvGpy+QGrI0B0adM\n"
"cs8p9rXU4O8TNk35w+kLbUxXm/Hp+65lcAd9o7BikBxCBntzM0BLfrYsutVCd5GU\n"
"CKGreByKmidQCz1KbdDNXUE2ywsY8WIacpaVmtvIPOaigjt3+A+CmPfkPc6cY2o3\n"
"9dvGkDPf5QKBgQC64+NibebfxLrtYc99dvCY8CGZLpTcaVShC8wf9UmMxsG30BuS\n"
"cKGqj6Mzp3Y1g8NfF7/wLRPKUPANXc4yZXcXW3bjyEwTZ3GNlHli8z6TEqjWzYEK\n"
"mbMCozZr1DIjjEn6CNNCizkqG+z+k8ZJIYnpaAwQdqXxVYOdVh5sm/KV/wKBgAZF\n"
"uMRaNSAPsZE7iTgY/thoKz37xxYn0YwZ3Y/OOy3JLbpwI7HypLHfqKjxEi8/gbyr\n"
"tL2OtsSqsv1Rvjv5atEWTpBVX+rlMSavf0xkgM2uvr/IJiNj1hlb0Ie3VnR/OMO7\n"
"aj27axa2oth1dRsnACeRM83P/qxy14gmQlLSmYn1AoGAAOvR+CJ5sh31zW0CnPQj\n"
"MsfURUxlk8h/F22MikFxIhnnDO5iQV1QITtsG5pgjKmw8XKWtWRKn5mNabP5yQ+w\n"
"PWJth0Z7ug2v4BW+VZetl75jExXBhR7l8MdAeatpYJpGb4NRSKMsOhYkDZwRnRez\n"
"7gIomK+77VVqHGZ/ykNI3dM=\n"
"-----END PRIVATE KEY-----\n";

#define DEFAULT_PORT 443

void handle_client(WOLFSSL* ssl) {
    char buffer[1024];
    int ret = wolfSSL_read(ssl, buffer, sizeof(buffer) - 1);
    if (ret > 0) {
        buffer[ret] = '\0';
        printf("Received: %s\n", buffer);

        const char* response = "HTTP/1.1 200 OK\r\n"
                               "Content-Type: application/json\r\n"
                               "Content-Length: 17\r\n"
                               "\r\n"
                               "{\"v1\": \"Redfish\"}";

        wolfSSL_write(ssl, response, strlen(response));
    }
}

int main() {
    WOLFSSL_CTX* ctx;
    WOLFSSL* ssl;
    int sockfd, clientfd;
    struct sockaddr_in servAddr, clientAddr;
    socklen_t clientLen = sizeof(clientAddr);

#ifdef DEBUG_WOLFSSH
    wolfSSL_Debugging_ON();
#endif

    wolfSSL_Init();

    ctx = wolfSSL_CTX_new(wolfTLSv1_2_server_method());
    if (ctx == NULL) {
        fprintf(stderr, "wolfSSL_CTX_new error.\n");
        return EXIT_FAILURE;
    }

    if (wolfSSL_CTX_use_certificate_buffer(ctx, server_cert, sizeof(server_cert), SSL_FILETYPE_PEM) != SSL_SUCCESS) {
        fprintf(stderr, "Error loading server certificate.\n");
        return EXIT_FAILURE;
    }

    if (wolfSSL_CTX_use_PrivateKey_buffer(ctx, server_key, sizeof(server_key), SSL_FILETYPE_PEM) != SSL_SUCCESS) {
        fprintf(stderr, "Error loading server key.\n");
        return EXIT_FAILURE;
    }

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        LOG_ERR("Socket error");
        return EXIT_FAILURE;
    }

    memset(&servAddr, 0, sizeof(servAddr));
    servAddr.sin_family = AF_INET;
    servAddr.sin_addr.s_addr = INADDR_ANY;
    servAddr.sin_port = htons(DEFAULT_PORT);

    if (bind(sockfd, (struct sockaddr*)&servAddr, sizeof(servAddr)) < 0) {
        LOG_ERR("Bind error");
        close(sockfd);
        return EXIT_FAILURE;
    }

    if (listen(sockfd, 5) < 0) {
        LOG_ERR("Listen error");
        close(sockfd);
        return EXIT_FAILURE;
    }

    printf("Server listening on port %d\n", DEFAULT_PORT);

    while (1) {
        clientfd = accept(sockfd, (struct sockaddr*)&clientAddr, &clientLen);
        if (clientfd < 0) {
            LOG_ERR("Accept error");
            continue;
        }

        ssl = wolfSSL_new(ctx);
        if (ssl == NULL) {
            fprintf(stderr, "wolfSSL_new error.\n");
            close(clientfd);
            continue;
        }

        wolfSSL_set_fd(ssl, clientfd);

        if (wolfSSL_accept(ssl) != SSL_SUCCESS) {
            fprintf(stderr, "wolfSSL_accept error.\n");
        } else {
            handle_client(ssl);
        }

        wolfSSL_shutdown(ssl);
        wolfSSL_free(ssl);
        close(clientfd);
    }

    wolfSSL_CTX_free(ctx);
    wolfSSL_Cleanup();
    close(sockfd);

    return 0;
}
