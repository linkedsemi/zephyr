# Linkedsemi LS ADC configuration options

# Copyright (c) 2025 Linkedsemi
# SPDX-License-Identifier: Apache-2.0

config ADC_LS
    bool "linkdesemi ls ADC driver"
    default y
    depends on DT_HAS_LINKEDSEMI_LS_ADC_ENABLED
    help
        Enable linkdesemi adc driver.