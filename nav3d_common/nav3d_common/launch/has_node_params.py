import launch
import yaml


class HasNodeParams(launch.Substitution):  # type: ignore[misc]
    def __init__(self, source_file: launch.SomeSubstitutionsType, node_name: str) -> None:
        super().__init__()

        from launch.utilities import normalize_to_list_of_substitutions

        self._source_file: list[launch.Substitution] = normalize_to_list_of_substitutions(source_file)
        self._node_name = node_name

    @property
    def name(self) -> list[launch.Substitution]:
        return self._source_file

    def describe(self) -> str:
        return ""

    def perform(self, context: launch.LaunchContext) -> str:
        yaml_filename = launch.utilities.perform_substitutions(context, self.name)
        with open(yaml_filename, encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        return "True" if self._node_name in data else "False"
