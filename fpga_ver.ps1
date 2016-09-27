

############################################
# CODE START
############################################
#Write-Host "Num Args:" $args.Length
#check arguments
foreach ($arg in $args)
{
	Write-Host "Arg: $arg";
}
	if(($args.length -lt 3) -or ($args[0] -eq '-h') -or ($arg[0] -eq '-?') -or ($arg[0] -eq '-help'))
	{
		Write-Host 	"------------------------------------------------"
					"Arguments: src_file dest_file version_str"
					"------------------------------------------------";
		exit
	}	
	
	$src_file = $args[0]
	$dst_file = $args[1]
	$ver_str = $args[2]

if(!(Test-Path -Path $src_file))
{
	Write-Host "ERROR: Cannot find path $src_file because it does not exist" -foregroundcolor red
	exit
}
if($ver_str.length -ne 8)
{
	Write-Host 'ERROR: Wrong version length - ' $ver_str.length ', must be 8' -foregroundcolor red
	exit
}	
[byte[]] $x = get-content -encoding byte -path $src_file

[byte[]] $y = 0xff,0x00
if(($x[0] -ne $y[0]) -or ($x[1] -ne $y[1]))
{
	Write-Host "ERROR: The source file is not valid." -foregroundcolor red
	exit
}
set-content -value $y -encoding byte -path $dst_file
add-content -value $ver_str -NoNewline -path $dst_file
$ind = $x.length - 1
add-content -value $x[2 .. $ind] -encoding byte -path $dst_file

