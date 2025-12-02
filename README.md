# SDFormat Website Documentation

This repository hosts the documentation content and scripts used to generate the
official `sdformat.org` website. It contains the source files for tutorials,
specifications, and other informational pages related to the SDFormat
(Simulation Description Format) project.

## Contributing

We welcome contributions to improve the SDFormat documentation!

To contribute documentation updates for `sdformat.org/tutorials`, you will primarily be editing or adding files within the `docs/` directory. If you add a new tutorial or a significant new page, ensure that the `docs/manifest.xml` file is also updated to include your new content.

All contributions should be made via Pull Requests. Please make sure to test your changes locally (see instructions below) before submitting a PR.

## Testing Locally

You can test your documentation changes locally using either Nanoc directly or
by building the provided Dockerfile.

### Option 1: Using Nanoc (Local Installation)

1. **Install Ruby and Bundler:** Ensure you have Ruby (version specified in
    `Gemfile`) and Bundler installed.
2. **Install Nanoc and dependencies:**

    ```bash
    bundle install
    ```

3. **Compile and view the site:**

    ```bash
    bundle nanoc view
    ```

    This will start a local web server, usually accessible at
    `http://localhost:3000`.

### Option 2: Using Docker

Using Docker allows for a consistent environment without installing Nanoc
directly on your host.

1. **Build the Docker image:**

    ```bash
    docker build -t sdf-docs .
    ```

2. **Run the Docker container with volume mounting for quicker iteration:** To
    enable quicker iteration, you can mount your local repository as a volume
    into the Docker container. This way, changes you make locally are
    immediately reflected in the container.

    ```bash
    docker run -p 3000:3000 -v "$(pwd):/usr/src/app" sdf-docs bundle nanoc view
    ```

    - `:/usr/src/app`: This is the path inside the Docker container where the repository
      contents will be mounted. The `Dockerfile` is configured to work within
      `/usr/src/app`.

    The site will be available in your browser at `http://localhost:3000`. Any
    changes saved locally will trigger a recompilation if `nanoc view` is
    running inside the container, and you can simply refresh your browser.

## For Maintainers

This section provides information relevant to repository maintainers.

### Site Deployment with GitHub Actions

The `sdformat.org` website is automatically built and deployed using GitHub
Actions.

- The workflow is defined in `.github/workflows/deploy.yml`.
- Upon merging changes into the `master` branch, the GitHub Action compiles the
  Nanoc site and deploys it to the production environment.

### Branch Protection with GitHub Rulesets

The `master` branch is protected using GitHub Rulesets to ensure code quality
and maintain a stable deployment.

- Direct pushes to `master` are prevented.
- All changes must be submitted via Pull Requests.
- Pull Requests require approval from at least one maintainer before merging.

