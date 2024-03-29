$version = ''

######### version S0 s-record
function vers_srec 
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
    Write-Host bytenum: $byteNum
    Write-Host $version.value
}

######### version S0 s-record
function get-version 
{
	param ( $ver_file_in, [ref]$version )

	#$version_file = get-content $ver_file_in

	$btld_or_app = (((Select-String -Path $ver_file_in -Pattern FW_VER_BTLD_OR_APP) -split 'x')[1].Trim())
	$fw_ver_major = (((Select-String -Path $ver_file_in -Pattern FW_VER_MAJOR) -split 'x')[1].Trim())
	$fw_ver_minor = (((Select-String -Path $ver_file_in -Pattern FW_VER_MINOR) -split 'x')[1].Trim())
	$fw_ver_build = (((Select-String -Path $ver_file_in -Pattern FW_VER_BUILD) -split 'x')[1].Trim())

	$version.value = "$btld_or_app.$fw_ver_major.$fw_ver_minor.$fw_ver_build"
    Write-Host $version.value
}

function ZipFiles( $zipfilename, $sourcedir )
{
   echo zipfilenam: $zipfilename
   echo sourcedir: $sourcedir
   Add-Type -Assembly System.IO.Compression.FileSystem
   $compressionLevel = [System.IO.Compression.CompressionLevel]::Optimal
   [System.IO.Compression.ZipFile]::CreateFromDirectory($sourcedir,
        $zipfilename, $compressionLevel, $false)
}


###### generate release
get-version  $PSScriptRoot'\MCU_TASKS\Sources\version.h' ([ref]$version )
Write-Host "version parsed: $version"
$folder_name = "NET869_MCU_$version"
$out_path = "$PSScriptRoot\out\*"
$release_path = "$PSScriptRoot\$folder_name"

if (Test-Path $release_path)
{
    Remove-Item -Path $release_path -Confirm:$false -Recurse
}

New-Item -Path $release_path -ItemType "directory" -Force
Copy-Item -Path "$out_path" -Destination $release_path -Force

Rename-Item -Path "$release_path\NET869_MCU_f0.srec" -NewName "NET869_MCU_f0_$version.srec" -Force
Rename-Item -Path "$release_path\NET869_MCU_f1.srec" "$release_path\NET869_MCU_f1_$version.srec" -Force

if ($PSVersionTable.PSVersion.Major -ge 4)
{
    if (Test-Path "$PSScriptRoot\$folder_name.zip")
	{
        Remove-Item -Path "$PSScriptRoot\$folder_name.zip" -Force
    }
    ZipFiles "$folder_name.zip" $release_path
    echo "release zip created at: $release_path.zip "
}
else
{
    echo "powershell version less than 4, folder created, please manually zip $release_path"
}

