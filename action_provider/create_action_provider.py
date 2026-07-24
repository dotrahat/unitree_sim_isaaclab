from action_provider.action_provider_dds import DDSActionProvider


from action_provider.action_provider_replay import FileActionProviderReplay

from action_provider.action_provider_wh_dds import DDSRLActionProvider
from action_provider.action_provider_wh_g123_dds import DDSRLActionProviderG123
from pathlib import Path


def create_action_provider(env,args):
    """create action provider based on parameters"""
    if args.action_source == "dds":
        return DDSActionProvider(
            env=env,
            args_cli=args
        )
    elif args.action_source == "dds_wholebody":
        if args.robot_type == "g123":
            return DDSRLActionProviderG123(
                env=env,
                args_cli=args
            )
        return DDSRLActionProvider(
            env=env,
            args_cli=args
        )
    elif args.action_source == "replay":
        return FileActionProviderReplay(env=env,args_cli=args)
    else:
        print(f"unknown action source: {args.action_source}")
        return None