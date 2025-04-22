################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"/Applications/ti/ccs2011/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --include_path="/Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/common/include" --include_path="/Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/headers/include" --include_path="/Applications/ti/ccs2011/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --advice:performance=all --define=_DEBUG --define=LARGE_MODEL --define=FLASH -g --diag_suppress=10063 --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_CodeStartBranch.obj: /Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/common/source/F2806x_CodeStartBranch.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"/Applications/ti/ccs2011/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --include_path="/Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/common/include" --include_path="/Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/headers/include" --include_path="/Applications/ti/ccs2011/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --advice:performance=all --define=_DEBUG --define=LARGE_MODEL --define=FLASH -g --diag_suppress=10063 --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_CpuTimers.obj: /Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/common/source/F2806x_CpuTimers.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"/Applications/ti/ccs2011/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --include_path="/Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/common/include" --include_path="/Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/headers/include" --include_path="/Applications/ti/ccs2011/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --advice:performance=all --define=_DEBUG --define=LARGE_MODEL --define=FLASH -g --diag_suppress=10063 --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_DefaultIsr.obj: /Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/common/source/F2806x_DefaultIsr.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"/Applications/ti/ccs2011/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --include_path="/Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/common/include" --include_path="/Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/headers/include" --include_path="/Applications/ti/ccs2011/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --advice:performance=all --define=_DEBUG --define=LARGE_MODEL --define=FLASH -g --diag_suppress=10063 --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_ECan.obj: /Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/common/source/F2806x_ECan.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"/Applications/ti/ccs2011/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --include_path="/Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/common/include" --include_path="/Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/headers/include" --include_path="/Applications/ti/ccs2011/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --advice:performance=all --define=_DEBUG --define=LARGE_MODEL --define=FLASH -g --diag_suppress=10063 --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_EPwm.obj: /Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/common/source/F2806x_EPwm.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"/Applications/ti/ccs2011/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --include_path="/Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/common/include" --include_path="/Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/headers/include" --include_path="/Applications/ti/ccs2011/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --advice:performance=all --define=_DEBUG --define=LARGE_MODEL --define=FLASH -g --diag_suppress=10063 --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_GlobalVariableDefs.obj: /Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/headers/source/F2806x_GlobalVariableDefs.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"/Applications/ti/ccs2011/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --include_path="/Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/common/include" --include_path="/Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/headers/include" --include_path="/Applications/ti/ccs2011/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --advice:performance=all --define=_DEBUG --define=LARGE_MODEL --define=FLASH -g --diag_suppress=10063 --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_PieCtrl.obj: /Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/common/source/F2806x_PieCtrl.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"/Applications/ti/ccs2011/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --include_path="/Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/common/include" --include_path="/Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/headers/include" --include_path="/Applications/ti/ccs2011/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --advice:performance=all --define=_DEBUG --define=LARGE_MODEL --define=FLASH -g --diag_suppress=10063 --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_PieVect.obj: /Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/common/source/F2806x_PieVect.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"/Applications/ti/ccs2011/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --include_path="/Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/common/include" --include_path="/Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/headers/include" --include_path="/Applications/ti/ccs2011/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --advice:performance=all --define=_DEBUG --define=LARGE_MODEL --define=FLASH -g --diag_suppress=10063 --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

F2806x_usDelay.obj: /Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/common/source/F2806x_usDelay.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"/Applications/ti/ccs2011/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --include_path="/Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/common/include" --include_path="/Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/headers/include" --include_path="/Applications/ti/ccs2011/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --advice:performance=all --define=_DEBUG --define=LARGE_MODEL --define=FLASH -g --diag_suppress=10063 --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

sicDCDC35kw.obj: ../sicDCDC35kw.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"/Applications/ti/ccs2011/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --include_path="/Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/common/include" --include_path="/Users/dugkyunshin/ti/c2000/C2000Ware_5_04_00_00/device_support/f2806x/headers/include" --include_path="/Applications/ti/ccs2011/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --advice:performance=all --define=_DEBUG --define=LARGE_MODEL --define=FLASH -g --diag_suppress=10063 --diag_warning=225 --issue_remarks --verbose_diagnostics --quiet --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


