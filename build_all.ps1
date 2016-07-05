
$iar_tool = "c:\Program Files `(x86`)\IAR Systems\Embedded Workbench 7.4\common\bin\iarbuild.exe"
$tproj = 'Release'
$tproj_m = 'release'
$core_num = 4
#-log errors|warnings|info|all	
$jar_log = 'info'
$out_path = "$PSScriptRoot\out"
$bl_path = "$PSScriptRoot\BootLoader\$tproj\Exe"
$app_file = "NET869_MCU.srec"
$app0_file = "NET869_MCU_0.srec"
$app1_file = "NET869_MCU_1.srec"
$full0_file = "NET869_MCU_f0.srec"
$full1_file = "NET869_MCU_f1.srec"
$bl_file = "BootLoader.srec"
$tmpl =  "FW_VER_BTLD_OR_APP", "FW_VER_MAJOR", "FW_VER_MINOR", "FW_VER_BUILD"
$vers = 'S00B0000'

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
	
	[byte]$byteNum 	= 0
	[uint32]$cs		= 0xB
	[string]$tmp = ""
	[string]$num_str = ""

	$file = get-content $file_in

	for($i=0; $i -lt 4; $i++)
	{
		$tmp = $tmpl[$i]
		$str = $file -match $tmp
		Write-Host $str;

		$num_str 		= "$(($str -split 'x')[1].Trim())"
		$byteNum 		= [convert]::ToByte($num_str, 16)
		$tmp 			= "{0:X2}" -f $byteNum

		$num_str		=  "{0:X2}" -f ([byte]$tmp[0])
		$version.value 	+= $num_str
		$num_str		=  "{0:X2}" -f ([byte]$tmp[1])
		$version.value 	+= $num_str
		$cs 			+= ([byte]$tmp[0] + [byte]$tmp[1])
	}
	$byteNum = (-bnot $cs) -band 0xFF  
	$tmp = "{0:X2}" -f $byteNum
	$version.value += $tmp
}
############################################
# CODE START
############################################
#Write-Host "Num Args:" $args.Length
#check arguments
foreach ($arg in $args)
{
#Write-Host "Arg: $arg";
	if($arg -eq '-nr')
	{ 
		$nr = $true
		Write-Host "-nr 		- Does not rebuild OS libraries";	
	}
	elseif($arg -eq '-bbl')
	{ 
		$bbl = $true
		Write-Host 	"-bbl		- Rebuilds BootLoader";
	}
	elseif(($arg -eq '-h') -or ($arg -eq '-?') -or ($arg -eq '-help'))
	{
		Write-Host 	"------------------------------------------------"
					"Optional arguments:"
					"------------------------------------------------"
					"-bbl		- Rebuilds BootLoader"
					"-nr 		- Does not rebuild OS libraries"
					"-help, -h, -?	- Help message"
					"------------------------------------------------";
		exit
	}	
}

#Added by Abid because script was not working if MCU project was not in C:\
Write-Host "psscript root: $PSScriptRoot";
$PSScriptRoot = $pwd
Write-Host "psscript root after change: $PSScriptRoot";
$out_path = "$PSScriptRoot\out"
$bl_path = "$PSScriptRoot\BootLoader\$tproj\Exe"

if(!(Test-Path -Path $out_path))
{
	New-Item -ItemType directory -Path $out_path
}	
else
{
	Remove-Item $out_path\*.*
}	
# New-Item $out_path PowerShell -type directory

#build version S0 s-record
#Write-Host 'building version S0 s-record';

vers_srec  $PSScriptRoot'\MCU_TASKS\Sources\version.h' ([ref]$vers )
Write-Host "version: $vers";

#rebuild os libs
if(!$nr)
{
	& $iar_tool 'MQX_OS\MQX_STDLIB\iar\mqx_stdlib_20d100m.ewp' -clean $tproj_m -log $jar_log -parallel $core_num
	& $iar_tool 'MQX_OS\MQX_STDLIB\iar\mqx_stdlib_20d100m.ewp' -make $tproj_m -log $jar_log -parallel $core_num

	& $iar_tool 'MQX_OS\MQX_USB_DEVICE\iar\usbd_sdk_mqx_lib_20d100m.ewp' -clean $tproj -log $jar_log -parallel $core_num
	& $iar_tool 'MQX_OS\MQX_USB_DEVICE\iar\usbd_sdk_mqx_lib_20d100m.ewp' -make $tproj -log $jar_log -parallel $core_num

	& $iar_tool 'MQX_OS\MQX_PSP\iar\mqx_psp_20d100m.ewp' -clean $tproj_m -log $jar_log -parallel $core_num
	& $iar_tool 'MQX_OS\MQX_PSP\iar\mqx_psp_20d100m.ewp' -make $tproj_m -log $jar_log -parallel $core_num

	& $iar_tool 'MQX_OS\MQX_LIB\iar\ksdk_mqx_20d100m_lib.ewp' -clean $tproj -log $jar_log -parallel $core_num
	& $iar_tool 'MQX_OS\MQX_LIB\iar\ksdk_mqx_20d100m_lib.ewp' -make $tproj -log $jar_log -parallel $core_num
}
# BootLoader
if($bbl)
{
	$bvers = 'S00B0000'
	vers_srec  $PSScriptRoot'\BootLoader\version.h' ([ref]$bvers )
	Write-Host "Bootloader Version: $bvers";
	& $iar_tool 'BootLoader\BootLoader.ewp' -clean $tproj -log $jar_log -parallel $core_num
	& $iar_tool "BootLoader\BootLoader.ewp" -make $tproj -log $jar_log	-parallel $core_num
#	Copy-Item BootLoader\Release\Exe\$bl_file $out_path\$bl_file
	copy_app_with_version $PSScriptRoot'\BootLoader\'$tproj'\Exe\'$bl_file $bl_file $bvers 
}
else
{
	Copy-Item $bl_path\$bl_file $out_path\$bl_file
}
#PFlash
Copy-Item MCU_TASKS\linker\files\MK20DX256xxx10_flash_0.icf MCU_TASKS\linker\MK20DX256xxx10_flash.icf

& $iar_tool 'MCU_TASKS\iar\NET869_MCU.ewp' -clean $tproj -log $jar_log -parallel $core_num
& $iar_tool 'MCU_TASKS\iar\NET869_MCU.ewp' -make $tproj -log $jar_log -parallel $core_num

# Copy-Item MCU_TASKS\iar\Release_IAR\$app_file $out_path\$app0_file
copy_app_with_version $PSScriptRoot'\MCU_TASKS\iar\'$tproj'_IAR\'$app_file $app0_file $vers 

#NVM flash
Copy-Item MCU_TASKS\linker\files\MK20DX256xxx10_flash_1.icf MCU_TASKS\linker\MK20DX256xxx10_flash.icf

& $iar_tool "MCU_TASKS\iar\NET869_MCU.ewp" -clean $tproj -log $jar_log	-parallel $core_num
& $iar_tool "MCU_TASKS\iar\NET869_MCU.ewp" -make $tproj -log $jar_log -parallel $core_num

#Copy-Item MCU_TASKS\iar\Release_IAR\$app_file $out_path\$app1_file
copy_app_with_version $PSScriptRoot'\MCU_TASKS\iar\'$tproj'_IAR\'$app_file $app1_file $vers 

# merging
cd $out_path
merge_srec $app0_file $full0_file "S1130800FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFCF7"
merge_srec $app1_file $full1_file "S31510000800FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFCE5"
cd $PSScriptRoot

