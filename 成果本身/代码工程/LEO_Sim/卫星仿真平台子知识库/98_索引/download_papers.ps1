$ErrorActionPreference = "Stop"

$vaultRoot = Split-Path -Parent $PSScriptRoot
$paperRoot = Get-ChildItem -Path $vaultRoot -Directory | Where-Object { $_.Name -like "03_*" } | Select-Object -First 1
if (-not $paperRoot) {
    throw "Cannot locate 03_* paper directory."
}
$pdfDir = Join-Path $paperRoot.FullName "PDF"
New-Item -ItemType Directory -Force -Path $pdfDir | Out-Null

$papers = @(
    @{
        Name = "01_A_Survey_on_Non_Geostationary_Satellite_Systems_The_Communication_Perspective.pdf"
        Url  = "https://arxiv.org/pdf/2107.05312.pdf"
    },
    @{
        Name = "02_LEO_Satellite_Access_Network_Towards_6G_The_Road_to_Space_Coverage.pdf"
        Url  = "https://arxiv.org/pdf/2207.11896.pdf"
    },
    @{
        Name = "03_Emerging_NGSO_Constellations_Spectral_Coexistence_with_GSO_Systems.pdf"
        Url  = "https://arxiv.org/pdf/2404.12651.pdf"
    },
    @{
        Name = "04_Evaluating_S_Band_Interference_Impact_of_Satellite_Systems_on_Terrestrial_Networks.pdf"
        Url  = "https://arxiv.org/pdf/2501.05462.pdf"
    },
    @{
        Name = "05_Null_Shaping_for_Interference_Mitigation_in_LEO_Satellites.pdf"
        Url  = "https://arxiv.org/pdf/2510.00816.pdf"
    },
    @{
        Name = "06_InfoGAN_Interpretable_Representation_Learning_by_Information_Maximizing_GANs.pdf"
        Url  = "https://arxiv.org/pdf/1606.03657.pdf"
    },
    @{
        Name = "07_Gradient_Based_Learning_Applied_to_Document_Recognition_LeNet.pdf"
        Url  = "http://yann.lecun.com/exdb/publis/pdf/lecun-98.pdf"
    },
    @{
        Name = "08_Hierarchical_Classification_Method_for_RFI_Recognition_and_Characterization_in_Satcom.pdf"
        Url  = "https://espace2.etsmtl.ca/id/eprint/21545/1/Landry-R-2020-21545.pdf"
    },
    @{
        Name = "09_RF_Based_Low_SNR_Classification_of_UAVs_Using_CNNs.pdf"
        Url  = "https://arxiv.org/pdf/2009.05519.pdf"
    },
    @{
        Name = "10_Modulation_Classification_Through_Deep_Learning_Using_Resolution_Transformed_Spectrograms.pdf"
        Url  = "https://arxiv.org/pdf/2306.04655.pdf"
    }
)

$results = foreach ($paper in $papers) {
    $outFile = Join-Path $pdfDir $paper.Name
    try {
        & curl.exe -L --fail --ssl-no-revoke --user-agent "Mozilla/5.0" $paper.Url -o $outFile | Out-Null
        $item = Get-Item $outFile
        [pscustomobject]@{
            File   = $paper.Name
            Status = "OK"
            Bytes  = $item.Length
            Url    = $paper.Url
        }
    } catch {
        if (Test-Path $outFile) {
            Remove-Item $outFile -Force -ErrorAction SilentlyContinue
        }
        [pscustomobject]@{
            File   = $paper.Name
            Status = "FAIL"
            Bytes  = 0
            Url    = $paper.Url
            Error  = $_.Exception.Message
        }
    }
}

$results | ConvertTo-Json -Depth 4 | Set-Content -Encoding utf8 (Join-Path $pdfDir "download_results.json")
$results | Format-Table -AutoSize
