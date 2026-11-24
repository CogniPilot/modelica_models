"""Library custody and runner argv controls; no compiler or Docker is invoked."""
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from tools import ci


class OpenModelicaLibraryTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="modelica-msl-runner-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name).resolve()
        self.msl = self.root / "pinned-msl"
        self.repository = self.root / "repository"
        self.repository.mkdir()
        for name in ("Modelica 4.1.0/package.mo",
                     "ModelicaServices 4.1.0/package.mo", "Complex.mo"):
            path = self.msl / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("synthetic library file\n")
        environment = mock.patch.dict(os.environ, {
            "MODELICA_MODELS_MSL": str(self.msl),
            "MODELICA_MODELS_DOCKER": "docker-test",
            "MODELICA_MODELS_OMC": "omc-test",
            "OPENMODELICALIBRARY": "/previous-libraries",
        }, clear=False)
        environment.start()
        self.addCleanup(environment.stop)
        for name, value in (("getuid", 1001), ("getgid", 1001)):
            patch = mock.patch.object(ci.os, name, return_value=value, create=True)
            patch.start()
            self.addCleanup(patch.stop)
        daemon = mock.patch.object(ci, "docker_daemon_is_available", return_value=True)
        self.daemon = daemon.start()
        self.addCleanup(daemon.stop)
        executable = mock.patch.object(ci, "executable_exists", return_value=True)
        executable.start()
        self.addCleanup(executable.stop)
        command = mock.patch.object(ci, "run_command")
        self.command = command.start()
        self.addCleanup(command.stop)

    def test_docker_receives_read_only_msl_as_the_existing_user(self):
        ci.run_omc_script(self.repository, "Tests/run.mos")
        argv = self.command.call_args.args[0]
        self.assertIn(f"{self.msl}:/msl:ro", argv)
        self.assertEqual(argv[argv.index("--user") + 1], "1001:1001")
        self.assertIn("OPENMODELICALIBRARY=/msl:/workspace", argv)
        self.assertEqual(argv[-3:], [ci.OPENMODELICA_IMAGE, "omc", "Tests/run.mos"])
        self.assertNotIn("/root/.openmodelica/libraries", " ".join(argv))

    def test_native_runner_uses_the_same_library_before_inherited_paths(self):
        self.daemon.return_value = False
        ci.run_omc_script(self.repository, "Tests/run.mos")
        self.assertEqual(self.command.call_args.args[0], ("omc-test", "Tests/run.mos"))
        environment = self.command.call_args.args[3]
        self.assertEqual(environment["OPENMODELICALIBRARY"].split(os.pathsep),
                         [str(self.msl), str(self.repository), "/previous-libraries"])

    def test_missing_configuration_refuses_before_dispatch(self):
        del os.environ["MODELICA_MODELS_MSL"]
        with self.assertRaisesRegex(ci.TaskError, "MSL 4.1.0 is required"):
            ci.run_omc_script(self.repository, "Tests/run.mos")
        self.command.assert_not_called()

    def test_missing_version_or_services_refuses_before_dispatch(self):
        for name in ("Modelica 4.1.0/package.mo", "ModelicaServices 4.1.0/package.mo"):
            with self.subTest(name=name):
                path = self.msl / name
                data = path.read_bytes()
                path.unlink()
                try:
                    with self.assertRaisesRegex(ci.TaskError, "not readable"):
                        ci.run_omc_script(self.repository, "Tests/run.mos")
                finally:
                    path.write_bytes(data)
        self.command.assert_not_called()

    def test_empty_library_refuses_before_dispatch(self):
        (self.msl / "Complex.mo").write_text("")
        with self.assertRaisesRegex(ci.TaskError, "file is empty"):
            ci.run_omc_script(self.repository, "Tests/run.mos")
        self.command.assert_not_called()
