
$iar_tool = "c:\Program Files `(x86`)\IAR Systems\Embedded Workbench 7.4\common\bin\iarbuild.exe"
$out_path = "$PSScriptRoot\MCU_TASKS\iar\out"
$bl_path = "$PSScriptRoot\BootLoader\Release\Exe"
$app_file = "NET869_MCU.srec"
$app0_file = "NET869_MCU_0.srec"
$app1_file = "NET869_MCU_1.srec"
$full0_file = "NET869_MCU_f0.srec"
$full1_file = "NET869_MCU_f1.srec"
$bl_file = "BootLoader.srec"
$tmpl =  "FW_VER_BTLD_OR_APP", "FW_VER_MAJOR", "FW_VER_MINOR", "FW_VER_BUILD"
$vers = 'S0070000'

######## merging bootloader and app with adding ready flag
Function merge_srec ($file_in, $file_out, $serc)
{
	(get-content $bl_file | Where-Object { $_ -notmatch '^S9.*$'}) | Set-Content $file_out
	$serc | Add-Content $file_out
	(get-content $file_in | Where-Object { $_ -notmatch '^S0.*$'}) | Add-Content $file_out
}
########
Function copy_app_with_version( $file_in, $file_out, $version) 
{
	$cont = get-content $file_in

	cd $out_path
	$version | Set-Content $file_out
	($cont | Where-Object { $_ -notmatch '^S0.*$'}) | Add-Content $file_out
	cd $PSScriptRoot
}
######### version S0 s-record
Function vers_srec 
{
	param ( $file_in, [ref]$version )
	
	[byte]$intNum
	[byte]$cs = 7
	$file = get-content $file_in
	[string]$tmp

	for($i=0; $i -lt 4; $i++)
	{
		$tmp = $tmpl[$i]
		$str = $file -match $tmp
		Write-Host "$str";
		$dig = "$(($str -split 'x')[1].Trim())"
		$intNum = [convert]::ToByte($dig, 16)

		$tmp = "{0:X2}" -f $intNum
		$version.value += $tmp
		$cs += $intNum
	}
	$intNum = (-bnot $cs) -band 0xFF  
	$tmp = "{0:X2}" -f $intNum
	$version.value += $tmp
}
############################################
# CODE START
############################################
Write-Host "Num Args:" $args.Length
#check arguments
foreach ($arg in $args)
{
	Write-Host "Arg: $arg";
	if($arg -eq '-nr')
	{ 
		$nr = $true	
	}
	elseif($arg -eq '-bbl')
	{ 
		$bbl = $true	
	}	
}

Remove-Item $out_path\*.*
# New-Item $out_path PowerShell -type directory

#build version S0 s-record
Write-Host "building version s-record"
vers_srec  $PSScriptRoot'\MCU_TASKS\Sources\version.h' ([ref]$vers )
Write-Host "version: $vers";

#rebuild os libs
if(!$nr)
{
	& $iar_tool 'MQX_OS\MQX_STDLIB\iar\mqx_stdlib_20d100m.ewp' -clean release -log info
	& $iar_tool 'MQX_OS\MQX_STDLIB\iar\mqx_stdlib_20d100m.ewp' -make release -log info

	& $iar_tool 'MQX_OS\MQX_USB_DEVICE\iar\usbd_sdk_mqx_lib_20d100m.ewp' -clean Release -log info
	& $iar_tool 'MQX_OS\MQX_USB_DEVICE\iar\usbd_sdk_mqx_lib_20d100m.ewp' -make Release -log info

	& $iar_tool 'MQX_OS\MQX_PSP\iar\mqx_psp_20d100m.ewp' -clean release -log info
	& $iar_tool 'MQX_OS\MQX_PSP\iar\mqx_psp_20d100m.ewp' -make release -log info

	& $iar_tool 'MQX_OS\MQX_LIB\iar\ksdk_mqx_20d100m_lib.ewp' -clean Release -log info
	& $iar_tool 'MQX_OS\MQX_LIB\iar\ksdk_mqx_20d100m_lib.ewp' -make Release -log info
}
# BootLoader
if($bbl)
{
	$bvers = 'S0070000'
	vers_srec  $PSScriptRoot'\BootLoader\version.h' ([ref]$bvers )
	& $iar_tool 'BootLoader\BootLoader.ewp' -clean Release -log info
	& $iar_tool "BootLoader\BootLoader.ewp" -make Release -log info	
#	Copy-Item BootLoader\Release\Exe\$bl_file $out_path\$bl_file
	copy_app_with_version $PSScriptRoot'\BootLoader\Release\Exe\'$bl_file $bl_file $bvers 
}
else
{
	Copy-Item $bl_path\$bl_file $out_path\$bl_file
}
#PFlash
Copy-Item MCU_TASKS\linker\files\MK20DX256xxx10_flash_0.icf MCU_TASKS\linker\MK20DX256xxx10_flash.icf

& $iar_tool 'MCU_TASKS\iar\NET869_MCU.ewp' -clean Release -log info
& $iar_tool 'MCU_TASKS\iar\NET869_MCU.ewp' -make Release -log info

# Copy-Item MCU_TASKS\iar\Release_IAR\$app_file $out_path\$app0_file
copy_app_with_version $PSScriptRoot'\MCU_TASKS\iar\Release_IAR\'$app_file $app0_file $vers 

#NVM flash
Copy-Item MCU_TASKS\linker\files\MK20DX256xxx10_flash_1.icf MCU_TASKS\linker\MK20DX256xxx10_flash.icf

& $iar_tool "MCU_TASKS\iar\NET869_MCU.ewp" -clean Release -log info	
& $iar_tool "MCU_TASKS\iar\NET869_MCU.ewp" -make Release -log info

#Copy-Item MCU_TASKS\iar\Release_IAR\$app_file $out_path\$app1_file
copy_app_with_version $PSScriptRoot'\MCU_TASKS\iar\Release_IAR\'$app_file $app1_file $vers 

# merging
cd $out_path
merge_srec $app0_file $full0_file "S1130800FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFCF7"
merge_srec $app1_file $full1_file "S31510000800FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFCE5"
cd $PSScriptRoot

