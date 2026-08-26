use core::convert::Infallible;

use rjmp_stm32_flash::{
    DualPageFlash, SectionUpdate, UpdateSource, load_sections, update_sections,
};
use zencan_node::common::NodeId;

#[repr(u8)]
enum FlashSection {
    NodeConfig = 1,
    Objects = 2,
    Unknown = 255,
}

impl From<u8> for FlashSection {
    fn from(value: u8) -> Self {
        match value {
            1 => Self::NodeConfig,
            2 => Self::Objects,
            _ => Self::Unknown,
        }
    }
}

pub fn store_objects<E>(
    flash: &mut dyn DualPageFlash<Error = E>,
    reader: &mut dyn embedded_io::Read<Error = Infallible>,
    size: usize,
) {
    if update_sections(
        flash,
        &mut [SectionUpdate {
            section_id: FlashSection::Objects as u8,
            data: UpdateSource::Reader((reader, size)),
        }],
    )
    .is_err()
    {
        defmt::error!("Error storing objects to flash");
    }
}

pub fn store_node_config<E>(flash: &mut dyn DualPageFlash<Error = E>, id: NodeId) {
    let data = [id.raw()];
    if update_sections(
        flash,
        &mut [SectionUpdate {
            section_id: FlashSection::NodeConfig as u8,
            data: UpdateSource::Slice(&data),
        }],
    )
    .is_err()
    {
        defmt::error!("Error storing node config to flash");
    }
}

pub fn read_saved_node_id<E>(flash: &dyn DualPageFlash<Error = E>, default: NodeId) -> NodeId {
    let Some(sections) = load_sections(flash) else {
        return default;
    };

    for section in sections {
        if matches!(
            FlashSection::from(section.section_id),
            FlashSection::NodeConfig
        ) {
            if let Some(raw) = section.data.first() {
                if let Ok(id) = NodeId::new(*raw) {
                    return id;
                }
                defmt::error!("Invalid node ID {} in flash", raw);
            } else {
                defmt::error!("Empty node configuration in flash");
            }
        }
    }
    default
}

pub fn read_persisted_objects<E>(flash: &dyn DualPageFlash<Error = E>, restore: impl Fn(&[u8])) {
    let Some(sections) = load_sections(flash) else {
        defmt::info!("No persistent data found in flash");
        return;
    };

    for section in sections {
        match FlashSection::from(section.section_id) {
            FlashSection::Objects => {
                defmt::info!("Loaded objects from flash");
                restore(section.data);
            }
            FlashSection::NodeConfig => {}
            FlashSection::Unknown => {
                defmt::warn!("Unrecognized flash section {}", section.section_id);
            }
        }
    }
}
