[CmdletBinding()]
param(
    [switch]$CheckMirrors
)

$ErrorActionPreference = 'Stop'
$failures = [System.Collections.Generic.List[string]]::new()
$skillRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$skillsDir = Split-Path $skillRoot -Parent
$agentsDir = Split-Path $skillsDir -Parent
$scopeRoot = Split-Path $agentsDir -Parent
$allowedRepos = @('flight_control', 'car_control', 'camera_helper')
$isInstalledLayout = ((Split-Path $skillsDir -Leaf) -eq 'skills') -and
    ((Split-Path $agentsDir -Leaf) -eq '.agents')

if($isInstalledLayout -and ((Split-Path $scopeRoot -Leaf) -in $allowedRepos))
{
    $suiteRoot = [System.IO.Path]::GetFullPath((Split-Path $scopeRoot -Parent))
}
elseif($isInstalledLayout)
{
    $suiteRoot = [System.IO.Path]::GetFullPath($scopeRoot)
}
else
{
    $suiteRoot = [System.IO.Path]::GetFullPath((Split-Path $skillRoot -Parent))
}

$sharedRoot = [System.IO.Path]::GetFullPath(
    (Join-Path $suiteRoot '.agents\skills\feiyue-leiqu'))
$legacyRoot = [System.IO.Path]::GetFullPath(
    (Join-Path $suiteRoot 'flight_control\.agents\skills\feiyue-leiqu'))

if(Test-Path -LiteralPath (Join-Path $sharedRoot 'SKILL.md'))
{
    $canonicalRoot = $sharedRoot
}
elseif(Test-Path -LiteralPath (Join-Path $legacyRoot 'SKILL.md'))
{
    $canonicalRoot = $legacyRoot
}
else
{
    $canonicalRoot = $skillRoot
}

$requiredFiles = @(
    'SKILL.md',
    'VERSION',
    'agents\openai.yaml',
    'references\onboarding.md',
    'references\system-contracts.md',
    'references\flight-architecture.md',
    'references\mission-development.md',
    'references\flight-control.md',
    'references\car-control.md',
    'references\vision-and-helper.md',
    'references\diagnosis-playbook.md',
    'references\validation-matrix.md',
    'references\evolution-protocol.md',
    'scripts\inspect-suite.ps1',
    'scripts\audit-skill.ps1',
    'scripts\sync-mirrors.ps1'
)

foreach($relativePath in $requiredFiles)
{
    if(-not (Test-Path -LiteralPath (Join-Path $canonicalRoot $relativePath)))
    {
        $failures.Add("Missing required file: $relativePath")
    }
}

$markdownFiles = @((Get-Item -LiteralPath (Join-Path $canonicalRoot 'SKILL.md')))
$markdownFiles += Get-ChildItem -LiteralPath (Join-Path $canonicalRoot 'references') -File -Filter '*.md'
$totalLines = 0
foreach($file in $markdownFiles)
{
    $content = Get-Content -LiteralPath $file.FullName -Raw
    $lineCount = (Get-Content -LiteralPath $file.FullName).Count
    $totalLines += $lineCount

    if(($file.Name -eq 'SKILL.md') -and ($lineCount -gt 160))
    {
        $failures.Add("SKILL.md exceeds 160 lines: $lineCount")
    }
    if(($file.Name -ne 'SKILL.md') -and ($lineCount -gt 220))
    {
        $failures.Add("Reference exceeds 220 lines: $($file.Name) ($lineCount)")
    }
    if($content -match '(?i)TODO|PLACEHOLDER|\[TODO')
    {
        $failures.Add("Unfinished placeholder in $($file.Name)")
    }
    if($content -match '[A-Za-z]:\\')
    {
        $failures.Add("Machine-local absolute path in $($file.Name)")
    }
}

if($totalLines -gt 1100)
{
    $failures.Add("Markdown instruction budget exceeded: $totalLines lines")
}

$skillFile = Join-Path $canonicalRoot 'SKILL.md'
if(Test-Path -LiteralPath $skillFile)
{
    $skillContent = Get-Content -LiteralPath $skillFile -Raw
    if($skillContent -notmatch '(?m)^name:\s*feiyue-leiqu\s*$')
    {
        $failures.Add('SKILL.md name must be feiyue-leiqu')
    }
    if($skillContent -notmatch '(?m)^description:\s*\S+')
    {
        $failures.Add('SKILL.md description is missing')
    }

    $referenceLinks = [regex]::Matches($skillContent, '\(references/([^\)]+)\)')
    foreach($match in $referenceLinks)
    {
        $referencePath = Join-Path $canonicalRoot ('references\' + $match.Groups[1].Value)
        if(-not (Test-Path -LiteralPath $referencePath))
        {
            $failures.Add("Broken reference link: $($match.Groups[1].Value)")
        }
    }
}

$versionFile = Join-Path $canonicalRoot 'VERSION'
if(Test-Path -LiteralPath $versionFile)
{
    $version = (Get-Content -LiteralPath $versionFile -Raw).Trim()
    if($version -notmatch '^\d+\.\d+\.\d+$')
    {
        $failures.Add("VERSION must use semantic x.y.z format: $version")
    }
}

if($CheckMirrors)
{
    foreach($repoName in $allowedRepos)
    {
        $mirrorRoot = Join-Path $suiteRoot "$repoName\.agents\skills\feiyue-leiqu"
        if(-not (Test-Path -LiteralPath $mirrorRoot))
        {
            $failures.Add("Mirror missing: $repoName")
            continue
        }

        foreach($relativePath in $requiredFiles)
        {
            $sourceFile = Join-Path $canonicalRoot $relativePath
            $mirrorFile = Join-Path $mirrorRoot $relativePath
            if(-not (Test-Path -LiteralPath $mirrorFile))
            {
                $failures.Add("Mirror file missing: $repoName/$relativePath")
                continue
            }
            $sourceHash = (Get-FileHash -LiteralPath $sourceFile -Algorithm SHA256).Hash
            $mirrorHash = (Get-FileHash -LiteralPath $mirrorFile -Algorithm SHA256).Hash
            if($sourceHash -ne $mirrorHash)
            {
                $failures.Add("Mirror differs: $repoName/$relativePath")
            }
        }
    }
}

if($failures.Count -gt 0)
{
    foreach($failure in $failures)
    {
        Write-Error $failure
    }
    exit 1
}

Write-Host "PASS feiyue-leiqu skill audit ($totalLines markdown lines)"
