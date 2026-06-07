"""Contract tests for clean_dot_labels in scripts/generate_pipeline_dotgraphs.py.

The fixture snippets are captured from real TaskComposerNode.dump() output
(tesseract_planning 0.34): node labels use `label="..."`, cluster labels use
`label = "..."` (spaced equals — missing that cost a regression once), and
annotated runs append Time / Status Code / Status Msg, the latter with raw
newlines inside the quoted value.
"""

import importlib.util
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
_SCRIPT = REPO_ROOT / "scripts" / "generate_pipeline_dotgraphs.py"

_spec = importlib.util.spec_from_file_location("generate_pipeline_dotgraphs", _SCRIPT)
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)
clean_dot_labels = _mod.clean_dot_labels

# Captured from FreespacePipeline dump() (annotated, failed run) — abridged UUIDs.
NODE_LABEL = (
    'node_633254f1 [shape=diamond, nojustify=true label="DiscreteContactCheckTask'
    "\\nType: tesseract::task_composer::DiscreteContactCheckTask"
    "\\lUUID: 633254f1-ff80-49ea-be0c-a3c82a5f766f"
    "\\lNamespace: DiscreteContactCheckTask"
    "\\lInputs:\\l\tprofiles: profiles\\l\tenvironment: environment"
    "\\l\tprogram: output_data\\lOutputs:"
    "\\lTime: 0.001s\\lStatus Code: 0\\lStatus Msg: Contact(s) found: \n"
    "Format: Step: [link_a, link_b] @ distance\n"
    'Step 0.0: [base_link, link_5] @ 0.5714\n\\l", color=red];'
)
CLUSTER_LABEL = (
    ' nojustify=true label = "FreespacePipeline'
    "\\nUUID: 2832e9e6-656c-4a6b-ba1f-d662028fe1b3"
    "\\lInputs:\\l\tenvironment: environment\\lOutputs:\\l\tprogram: output_data"
    '\\lAbort Terminal: -1\\lConditional: False\\lTime: 0.075s\\l";'
)
EDGE_LABEL = 'node_a -> node_b [style=dashed, label="[1]"];'

DOT_SNIPPET = "\n".join((NODE_LABEL, CLUSTER_LABEL, EDGE_LABEL))


class TestCleanDotLabels:
    def test_debug_fields_dropped(self):
        cleaned = clean_dot_labels(DOT_SNIPPET)
        for noise in (
            "Type:",
            "UUID:",
            "Namespace:",
            "Abort Terminal:",
            "Conditional:",
            "Status Code:",
            "Inputs:",
            "Outputs:",
            "profiles: profiles",
            "environment: environment",
        ):
            assert noise not in cleaned, f"{noise!r} leaked through"

    def test_task_names_kept(self):
        cleaned = clean_dot_labels(DOT_SNIPPET)
        assert 'label="DiscreteContactCheckTask\\n' in cleaned
        assert 'label="FreespacePipeline\\n' in cleaned

    def test_cluster_spaced_equals_form_cleaned(self):
        # Regression guard: cluster labels are `label = "..."`, not `label="..."`.
        cleaned = clean_dot_labels(CLUSTER_LABEL)
        assert "UUID:" not in cleaned
        assert "FreespacePipeline" in cleaned

    def test_annotated_run_fields_kept(self):
        cleaned = clean_dot_labels(DOT_SNIPPET)
        assert "Time: 0.001s" in cleaned
        assert "Time: 0.075s" in cleaned
        # Status Msg survives including its raw-newline contact details
        assert "Status Msg: Contact(s) found:" in cleaned
        assert "[base_link, link_5] @ 0.5714" in cleaned

    def test_single_segment_labels_unchanged(self):
        assert clean_dot_labels(EDGE_LABEL) == EDGE_LABEL

    def test_structure_untouched(self):
        cleaned = clean_dot_labels(DOT_SNIPPET)
        for token in (
            "node_633254f1 [shape=diamond, nojustify=true ",
            ", color=red];",
            "node_a -> node_b [style=dashed, ",
            " nojustify=true ",
        ):
            assert token in cleaned

    def test_idempotent(self):
        once = clean_dot_labels(DOT_SNIPPET)
        assert clean_dot_labels(once) == once
