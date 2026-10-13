"""Diagnostics are explicit generated bindings, separate from rejected payloads."""
from luggage_msgs.msg import BoundaryDiagnosticV2
from luggage_description.frame_diagnostics import diagnostic


def diagnostic_to_message(record, context, error=None):
    fields = diagnostic(record, context, error)
    message = BoundaryDiagnosticV2()
    for name, value in fields.items():
        setattr(message, name, value)
    return message
