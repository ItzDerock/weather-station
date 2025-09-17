# Set usual component variables
set(COMPONENT_SRCS "argentdata.c")
set(COMPONENT_ADD_INCLUDEDIRS "include")
set(COMPONENT_REQUIRES ulp driver esp_timer esp_adc)

register_component()

# ULP -------------
set(ulp_app_name ulp_argentdata)
set(ulp_riscv_sources "ulp/main.c")

# Ensures the ULP's header file is available before building these files.
set(ulp_exp_dep_srcs "argentdata.c")
ulp_embed_binary(${ulp_app_name} "${ulp_riscv_sources}" "${ulp_exp_dep_srcs}")
