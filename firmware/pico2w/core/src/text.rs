//! Hardware-independent text helpers shared with the firmware console.

/// Return the part of a command line after its first whitespace token.
///
/// The debug bridge forwards this remainder to the mainboard verbatim; using
/// only the next token would truncate commands such as `$GET ANG_MEAS`.
pub fn command_tail(text: &str) -> &str {
    let text = text.trim();
    match text.find(char::is_whitespace) {
        Some(index) => text[index..].trim_start(),
        None => "",
    }
}

#[cfg(test)]
mod tests {
    use super::command_tail;

    #[test]
    fn single_token_has_no_tail() {
        assert_eq!(command_tail("ping"), "");
        assert_eq!(command_tail("  ping  "), "");
    }

    #[test]
    fn keeps_the_whole_remainder() {
        assert_eq!(command_tail("dbg $GET ANG_MEAS"), "$GET ANG_MEAS");
        assert_eq!(command_tail("dbg $SET FILT 65535"), "$SET FILT 65535");
    }

    #[test]
    fn trims_leading_whitespace() {
        assert_eq!(command_tail("dbg \t $GET  FILT"), "$GET  FILT");
    }
}
