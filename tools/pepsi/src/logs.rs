use std::{path::PathBuf, sync::Arc};

use bat::{Input, PrettyPrinter};
use clap::Subcommand;
use color_eyre::{eyre::WrapErr, Result};

use futures_util::stream::{FuturesOrdered, StreamExt};
use nao::Nao;

use crate::{parsers::NaoAddress, progress_indicator::ProgressIndicator};

#[derive(Subcommand)]
pub enum Arguments {
    // Delete logs on the NAOs
    Delete {
        /// The NAOs to delete logs from e.g. 20w or 10.1.24.22
        #[arg(required = true)]
        naos: Vec<NaoAddress>,
    },
    // Download logs from the NAOs
    Download {
        /// Directory where to store the downloaded logs (will be created if not existing)
        log_directory: PathBuf,
        /// The NAOs to download logs from e.g. 20w or 10.1.24.22
        #[arg(required = true)]
        naos: Vec<NaoAddress>,
    },
    // Show logs from the NAOs
    Show {
        /// The NAOs to delete logs from e.g. 20w or 10.1.24.22
        #[arg(required = true)]
        naos: Vec<NaoAddress>,
    },
}

pub async fn logs(arguments: Arguments) -> Result<()> {
    match arguments {
        Arguments::Delete { naos } => {
            ProgressIndicator::map_tasks(naos, "Deleting logs...", |nao_address| async move {
                let nao = Nao::new(nao_address.ip);
                nao.delete_logs()
                    .await
                    .wrap_err_with(|| format!("failed to delete logs on {nao_address}"))
            })
            .await
        }
        Arguments::Download {
            log_directory,
            naos,
        } => {
            ProgressIndicator::map_tasks(naos, "Downloading logs...", |nao_address| {
                let log_directory = log_directory.join(nao_address.to_string());
                async move {
                    let nao = Nao::new(nao_address.ip);
                    nao.download_logs(log_directory)
                        .await
                        .wrap_err_with(|| format!("failed to download logs from {nao_address}"))
                }
            })
            .await
        }
        Arguments::Show { naos } => {
            for nao_address in naos {
                let nao = Arc::new(Nao::new(nao_address.ip));
                let log_files = nao.ls_logs().await?;

                let log_contents: Vec<String> = log_files
                    .iter()
                    .map(|path| {
                        let nao = nao.clone();
                        async move { nao.get_file(path).await.wrap_err("failed to read log file") }
                    })
                    .collect::<FuturesOrdered<_>>()
                    .collect::<Vec<_>>()
                    .await
                    .into_iter()
                    .collect::<Result<_, _>>()?;

                for (log_path, log_content) in log_files.iter().zip(log_contents.into_iter()) {
                    PrettyPrinter::new()
                        .header(true)
                        .grid(true)
                        .line_numbers(true)
                        .use_italics(true)
                        .input(Input::from_bytes(log_content.as_bytes()).name(log_path.as_path()))
                        .print()?;
                }
            }
        }
    }

    Ok(())
}
