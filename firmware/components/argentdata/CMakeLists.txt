# Set usual component variables
set(COMPONENT_SRCS "argentdata.c")
set(COMPONENT_ADD_INCLUDEDIRS "include")
set(COMPONENT_REQUIRES ulp driver)

register_component()

# ULP -------------
set(ulp_app_name ulp_argentdata)
set(ulp_riscv_sources "ulp/main.c")

# Ensures the ULP's header file is available before building these files.
set(ulp_exp_dep_srcs "argentdata.c")
ulp_embed_binary(${ulp_app_name} "${ulp_riscv_sources}" "${ulp_exp_dep_srcs}")

# OLD CONFIG
# set(COMPONENT_REQUIRES ulp)
#
# idf_component_register(SRCS "argentdata.c"
#                     INCLUDE_DIRS "include"
#                       "${IDF_PATH}/components/ulp/ulp_riscv/include"
#                       "${IDF_PATH}/components/ulp/ulp_riscv/ulp_core/include"
#                       "${IDF_PATH}/components/ulp/ulp_riscv/shared/include"
#                       "${IDF_PATH}/components/ulp/ulp_common/include/"
#                     )
#
# set(ulp_app_name ulp_argentdata)
# set(ulp_sources "ulp/main.c")
# set(ulp_exp_dep_srcs "argentdata.c")
#
# ulp_embed_binary(${ulp_app_name} "${ulp_sources}" "${ulp_exp_dep_srcs}")
#
