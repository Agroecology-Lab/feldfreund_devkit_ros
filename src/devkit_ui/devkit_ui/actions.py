from dataclasses import dataclass


@dataclass(frozen=True)
class ActionDef:
    """Definition of a mission action (drive, weed, etc.)."""
    label: str
    icon: str
    tool_topic: str | None = None
    param_schema: tuple = ()  # Optional parameter definitions for the action

ACTIONS: dict[str, ActionDef] = {
    'drive': ActionDef(label='Drive only', icon='🚜', tool_topic=None),
    'weed':  ActionDef(label='Weed',       icon='🌿', tool_topic='/tool/weeder/enable'),
}
