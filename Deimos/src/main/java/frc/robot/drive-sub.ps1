# Powershell regex to extract navx reports "MNAVX=[123.456]" from a file

# C:\Users\Public\Documents\FRC\Log Files\DSLogs\
$file = "path/to/your/file.txt"
$matches = [regex]::Matches((Get-Content $file -Raw, 'MNAVX=\[(\d+\.\d+)\]'))

# $inputString = @" 
# MNAVX=[2939484828.4949] 
# MNAVX=[3939292.4] 
# UselessData 
# "@
# $matches = [regex]::Matches($inputString, 'MNAVX=\[(\d+\.\d+)\]')
$output = foreach ($match in $matches) { $match.Groups[1].Value }
$output -join ','